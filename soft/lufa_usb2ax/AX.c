/* ****************************************************************************
Copyright (C) 2012 Nicolas Saugnier (nicolas.saugnier [at] esial {dot} net),
                   Richard Ibbotson (richard.ibbotson [at] btinternet {dot} com)

Date   : 2012/05/06

Based on :
    - USBtoSerial project in the LUFA Library by Dean Camera (original file)
    - USB2AXSYNC by Richard Ibbotson (USB2AX optimizations, sync_read and Dynamixel communication)
    - arbotix_python ROS package and Arbotix Robocontroller firmware by Michael Ferguson (sync_read and part of the Dynamixel communication)

Original copyright notice : 
  Copyright 2012  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*******************************************************************************/

#include "AX.h" 

extern RingBuffer_t ToUSB_Buffer;

// BUGBUG:: Move to appropriate locations
uint8_t g_bRegReturnVal = 2;

// COULD do with union...
// Voltage Pin, Frame length in ms, Interpolating, pose size, ids...
uint8_t g_abPoseRegVals[AX_REG_POSE_LAST_REG-AX_REG_POSE_FIRST_REG + 1] = {0, BIOLOID_FRAME_LENGTH, 0, 31 };
#define GETPOSEREGVAL(reg) (g_abPoseRegVals[(reg) - AX_REG_POSE_FIRST_REG])
// For AX12 like servos max values are 1023 if we shift left 3 bits still no problem fitting.
// for MX servos I believe max is 4095, so again should fit when shifted left 3, with room for sign bit...
typedef struct _poseinfo {
	uint16_t		pose_;
	uint16_t		next_pose_;
	uint16_t		speed_;
	} POSEINFO;

POSEINFO g_aPoseinfo[AX_SYNC_READ_MAX_DEVICES];	
uint16_t g_wPoseNextFrameTimeout = BIOLOID_FRAME_LENGTH*TIMER_TICS_PER_MS;

// forward references
extern void ProcessGetVoltageRequest(void);

/* 
 * Initialize our ax registers and the like
 */
void AXInit() {
	for (uint8_t i=0; i < AX_SYNC_READ_MAX_DEVICES; i++) {
		g_aPoseinfo[i].pose_ = 512 << BIOLOID_SHIFT;		// shift out to give more room to interpolate.
		g_aPoseinfo[i].next_pose_ = 512 << BIOLOID_SHIFT;
		g_aPoseinfo[i].speed_ = 0;
		GETPOSEREGVAL(i+AX_REG_POSE_ID_FIRST) = i + 1;
	}
}


/*
 * Send status packet
 */
void axStatusPacket(uint8_t err, uint8_t* data, uint8_t nb_bytes){
	uint16_t checksum = AX_ID_DEVICE + 2 + nb_bytes + err;

	if (g_bRegReturnVal == 2) {
		cdc_send_byte(0xff);
		cdc_send_byte(0xff);
		cdc_send_byte(AX_ID_DEVICE);
		cdc_send_byte(2 + nb_bytes);
		cdc_send_byte(err);
		for (uint8_t i = 0; i < nb_bytes; i++){
    		cdc_send_byte(data[i]);
			checksum += data[i];
		}
		cdc_send_byte(255-(checksum %256));
	}
}


// try to read a Dynamixel packet
// return true if successful, false otherwise 
uint16_t axReadPacket(uint8_t length){
    setRX();
    usart_timer = 0;
	// wait until the expected number of byte has been read
	while( local_rx_buffer_count < length ){ 
		if(usart_timer > USART_TIMEOUT){
		    break;
		}
	}
	setTX();
	if (local_rx_buffer_count != length){
		return false;
	}
	
	// TODO check for error in the packet, so that we don't wait if the status packet says that something went wrong... ?
	
	// compute checksum
    uint8_t checksum = 0; // accumulator for checksum
    for(uint8_t i=2; i < length; i++){
        checksum += local_rx_buffer[i];
    }
    
    if((checksum%256) != 255){
        return false; // invalid checksum
    }else{
        return true;
    }
}


/** Read register value(s) */
int axGetRegister(uint8_t id, uint8_t addr, uint8_t nb_bytes){  
   // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM    
    uint16_t checksum = ~((id + 6 + addr + nb_bytes)%256);

	local_rx_buffer_count = 0;
	serial_write(0xFF);
    serial_write(0xFF);
    serial_write(id);
    serial_write(4);    // length
    serial_write(AX_CMD_READ_DATA);
    serial_write(addr);
    serial_write(nb_bytes);
    serial_write(checksum);

    return axReadPacket(nb_bytes + 6);
}


// sync_read performs a cycle of Dynamixel reads to collect the data from servos to return over USB
void sync_read(uint8_t* params, uint8_t nb_params){
    
    // divert incoming data to a buffer for local processing  
	passthrough_mode = AX_DIVERT;
	
	uint8_t addr = params[0];    // address to read in control table
    uint8_t nb_to_read = params[1];    // # of bytes to read from each servo
    uint8_t nb_servos = nb_params - 2;
	
	cdc_send_byte(0xff);
	cdc_send_byte(0xff);
	cdc_send_byte(AX_ID_DEVICE);
	cdc_send_byte(2+(nb_to_read*nb_servos));
	cdc_send_byte(0);  //error code
    
	// get ax data
	uint8_t fcount = 5; // count of number of bytes in USB IN endpoint buffer
	uint8_t checksum = AX_ID_DEVICE + (nb_to_read*nb_servos) + 2;  // start accumulating the checksum
    uint8_t* servos = params + 2; // pointer to the ids of the servos to read from
	uint8_t* data = local_rx_buffer + 5; // pointer to the parameters in the packet received from the servo
	for(uint8_t servo_id = 0; servo_id < nb_servos; servo_id++){
        if( axGetRegister(servos[servo_id], addr, nb_to_read)){
            for(uint8_t i = 0; i < nb_to_read; i++){
                checksum += data[i];
                cdc_send_byte(data[i]);
				if(fcount++ >= CDC_TXRX_EPSIZE){ // periodically try to flush data to the host
					send_USB_data();
                    fcount = 0;
                }
            }
        } else{
            for(uint8_t i = 0; i < nb_to_read; i++){
                checksum += 0xFF;
                cdc_send_byte(0xFF);
				if(fcount++ >= CDC_TXRX_EPSIZE){ 
					send_USB_data();
                    fcount = 0;
                }
            }
		}
    }
	
    cdc_send_byte(255-((checksum)%256));
 	
	// allow data from USART to be sent directly to USB
	passthrough_mode = AX_PASSTHROUGH;
}


void local_read(uint8_t addr, uint8_t nb_bytes){
	// currently, local read only supports registers for Model Number and Version of Firmware
	// Added support for Return level, number of servos, and servo ids...
	uint8_t regs[] = {MODEL_NUMBER_L, MODEL_NUMBER_H, FIRMWARE_VERSION, AX_ID_DEVICE};
	
	uint16_t top = (uint16_t)addr + nb_bytes;
	if ( top < sizeof(regs) ){
		axStatusPacket(AX_ERROR_NONE, regs + addr, nb_bytes);

	} else if ((addr == AX_REG_RETURN_LEVEL) && (nb_bytes == 1)) {
		axStatusPacket(AX_ERROR_NONE, &g_bRegReturnVal, 1);

	// Special case voltage, to return a cached voltage (if any)
	} else if ((addr == AX_REG_VOLTAGE) && (nb_bytes == 1)) {
		ProcessGetVoltageRequest();

	} else if ((addr >= AX_REG_POSE_FIRST_REG) && (top <= AX_REG_POSE_LAST_REG)) {
		axStatusPacket(AX_ERROR_NONE, &GETPOSEREGVAL(addr), nb_bytes);

	} else if ((addr >= AX_REG_SLOT_CUR_POSE_FIRST) && (top <= AX_REG_SLOT_CUR_POSE_LAST) && (nb_bytes == 2)) {
		// Hack way to be able to retrieve the current value from my pose engine...
		uint8_t abCurPose[2];
		uint16_t wPose = (g_aPoseinfo[(addr-AX_REG_SLOT_CUR_POSE_FIRST)/2].pose_) >> BIOLOID_SHIFT;	// should verify even number but...
		abCurPose[0] = wPose & 0xff;
		abCurPose[1] = wPose >> 8; 
		axStatusPacket(AX_ERROR_NONE, abCurPose, nb_bytes);
	} else
		axStatusPacket( AX_ERROR_RANGE, NULL, 0 );
	
}

void local_write(uint8_t addr, uint8_t nb_bytes, uint8_t *pb){
	uint16_t top = (uint16_t)addr + nb_bytes;
	
#ifdef DEBUG
	hwbb_toggle(PORTB6);
#endif
	// BUGBUG:: not currently supporting setting the version values and the like.
	if ((addr == AX_REG_RETURN_LEVEL) && (nb_bytes == 1)) {
		g_bRegReturnVal = *pb;	// should do some validation...
		axStatusPacket(AX_ERROR_NONE, 0, 0);
		
	} else if ((addr >= AX_REG_POSE_FIRST_REG) && (top <= AX_REG_POSE_LAST_REG)) {
		// Update the logical registers to the new values
		uint8_t *pbReg = &GETPOSEREGVAL(addr);
		while (nb_bytes--)
			*pbReg++ = *pb++;
		axStatusPacket(AX_ERROR_NONE, 0, 0);
			
	} else
		axStatusPacket( AX_ERROR_RANGE, NULL, 0 );
}

/*
 * ProcessReadPoseCmd - This command tells the system to go out and query the servos for their current positions and
 *		fill in our pose table.  Also this command
 * has the side effect of stopping any moves that happen to be going on when started. 
 */
void ProcessReadPoseCmd(uint8_t nb_bytes, uint8_t *pb) {
	
	// Pass 1, cancel all, next pass look at PB for servo list to cancel
    // divert incoming data to a buffer for local processing
	// Tell the system we need the bytes ourself, don't pass on to the host
    passthrough_mode = AX_DIVERT;
	for (uint8_t iSlot = 0; iSlot < GETPOSEREGVAL(AX_REG_POSE_SIZE); iSlot++) {
		// We need to load in the current position from the actual servo.
		// Need to better abstract where we get the ID from.
		if (axGetRegister(GETPOSEREGVAL(iSlot+AX_REG_POSE_ID_FIRST), AX_GOAL_POSITION_L, 2)) {
			g_aPoseinfo[iSlot].pose_ = ((uint16_t)local_rx_buffer[5] + ((uint16_t)local_rx_buffer[6] << 8)) << BIOLOID_SHIFT;
		} else
			g_aPoseinfo[iSlot].pose_ = (uint16_t)512 << BIOLOID_SHIFT;	// If read fails init to something...

		g_aPoseinfo[iSlot].next_pose_ = g_aPoseinfo[iSlot].pose_ ;
		g_aPoseinfo[iSlot].speed_ = 0;
	}
	GETPOSEREGVAL(AX_REG_POSE_INTERPOLATING) = 0;	// Clear out the Interpolating state.
	passthrough_mode = AX_PASSTHROUGH;
	axStatusPacket(AX_ERROR_NONE, 0, 0);
}


/* 
 * This Pose command allows us to pass in three bytes per servo, the ID followed by the two byte next_pos_ value
 * This version of the command is nice for simple moves. Pass 2 will allow us to specify specific ids.  
 */
void ProcessPoseIDsCmd(uint8_t nb_bytes, uint8_t *pb) {
#ifdef DEBUG_POSE
	hwb_toggle(PORTD4);
	uint8_t abInfoRet[20];
	uint8_t cbInfoRet = 0;
#endif
	// First 2 bytes is the move time in ms
	if (nb_bytes < 2) {
		axStatusPacket( AX_ERROR_RANGE, NULL, 0 );
		return;
	}

	int8_t cServosInterpolating = GETPOSEREGVAL(AX_REG_POSE_INTERPOLATING);
	uint16_t wMoveTime = *pb + (uint16_t)(*(pb+1) << 8);
	uint16_t wMoveIters = (wMoveTime / GETPOSEREGVAL(AX_REG_POSE_FRAME_TIME)) + 1;
	pb +=2;
	nb_bytes -= 2;	//
	uint8_t err = AX_ERROR_NONE;

#ifdef DEBUG_POSE
	abInfoRet[cbInfoRet++] = wMoveIters & 0xff;
	abInfoRet[cbInfoRet++] = wMoveIters >> 8;
#endif

	// Now lets walk through the data extracting servos and positions
	// Try to hack in some performance helper if the user passes in the
	// servos in the order of the slots...
	uint8_t iSlot = 0xff;
	while (nb_bytes >= 3) {
		// Next byte should be a servo number we wish to update.
		iSlot++;
		if (*pb != GETPOSEREGVAL(iSlot+AX_REG_POSE_ID_FIRST)) {
			for (iSlot = 0; iSlot < GETPOSEREGVAL(AX_REG_POSE_SIZE); iSlot++) {
				if (*pb == GETPOSEREGVAL(iSlot+AX_REG_POSE_ID_FIRST))
					break;
			}
		}
		if (iSlot >= GETPOSEREGVAL(AX_REG_POSE_SIZE))		
			axStatusPacket( AX_ERROR_RANGE, NULL, 0 );	// did not find that servo
		g_aPoseinfo[iSlot].next_pose_ = ((uint16_t)(pb[1] + ((uint16_t)(pb[2]) << 8)) << BIOLOID_SHIFT);
		pb += 3;
		nb_bytes -= 3;
		
		// Compute Speed.
		if (g_aPoseinfo[iSlot].speed_)
		cServosInterpolating--;	// was already moving so don't add twice.
		// Handle case where we already had a speed...
		if (g_aPoseinfo[iSlot].next_pose_ != g_aPoseinfo[iSlot].pose_) {
			if(g_aPoseinfo[iSlot].next_pose_ > g_aPoseinfo[iSlot].pose_) {
				g_aPoseinfo[iSlot].speed_ = (g_aPoseinfo[iSlot].next_pose_ - g_aPoseinfo[iSlot].pose_) / wMoveIters + 1;
				} else {
				g_aPoseinfo[iSlot].speed_ = (g_aPoseinfo[iSlot].pose_ - g_aPoseinfo[iSlot].next_pose_) / wMoveIters + 1;
			}
			cServosInterpolating++;
		} else
			g_aPoseinfo[iSlot].speed_ = 0;
#ifdef DEBUG_POSE
		abInfoRet[cbInfoRet++] = iSlot;
		abInfoRet[cbInfoRet++] = GETPOSEREGVAL(iSlot+AX_REG_POSE_ID_FIRST);
		abInfoRet[cbInfoRet++] = g_aPoseinfo[iSlot].pose_ & 0xff;
		abInfoRet[cbInfoRet++] = ((uint16_t)g_aPoseinfo[iSlot].pose_)>>8;
		abInfoRet[cbInfoRet++] = g_aPoseinfo[iSlot].next_pose_ & 0xff;
		abInfoRet[cbInfoRet++] = ((uint16_t)g_aPoseinfo[iSlot].next_pose_)>>8;
		abInfoRet[cbInfoRet++] = g_aPoseinfo[iSlot].speed_ & 0xff;
		abInfoRet[cbInfoRet++] = ((uint16_t)g_aPoseinfo[iSlot].speed_)>>8;
#endif
	}

	// If we are now starting a new move 0 out the move timer and calculate a good timeout
	if (cServosInterpolating < 0)
		cServosInterpolating = 0;
	if (!GETPOSEREGVAL(AX_REG_POSE_INTERPOLATING)) {
		pose_timer = 0;	// timer for processing poses.
		g_wPoseNextFrameTimeout =  GETPOSEREGVAL(AX_REG_POSE_FRAME_TIME)*TIMER_TICS_PER_MS;
		// fudge factor each servo adds 3 bytes (.01 second per byte) so to have commands end at the same time,
		// we fudge our start time to deal with this...
		if (cServosInterpolating)
			g_wPoseNextFrameTimeout += ((uint16_t)(GETPOSEREGVAL(AX_REG_POSE_SIZE) - cServosInterpolating) * 3)/2;
	}
	GETPOSEREGVAL(AX_REG_POSE_INTERPOLATING) = cServosInterpolating;   // don't clear if other move is happening
#ifdef DEBUG_POSE
	abInfoRet[cbInfoRet++] = (uint8_t)cServosInterpolating;
	abInfoRet[cbInfoRet++] = g_abPoseRegVals[AX_REG_POSE_INTERPOLATING-AX_REG_POSE_FIRST_REG];
	abInfoRet[cbInfoRet++] = pose_timer & 0xff;
	abInfoRet[cbInfoRet++] = pose_timer >> 8;
	axStatusPacket(err, abInfoRet, cbInfoRet);
#else
	axStatusPacket(err, 0, 0);
#endif
}

/* 
 * This Pose command allows us to pass in a mask of which slots we wish to start a move with.
 * For each bit that is set in the mask, there will be a two byte next_pose_ value specified.
 * This version of the command is good for large moves.
 */
void ProcessPoseMaskCmd(uint8_t nb_bytes, uint8_t *pb) {
#ifdef DEBUG_POSE
	hwb_toggle(PORTD4);
	uint8_t abInfoRet[20];
	uint8_t cbInfoRet = 0;
#endif
// First 2 bytes is the move time in ms
	// next 5 bytes is a mask of which slots there is data passed to us. 
	if (nb_bytes < 7) {
		axStatusPacket( AX_ERROR_RANGE, NULL, 0 );
		return;
	}	
	
	int8_t cServosInterpolating = GETPOSEREGVAL(AX_REG_POSE_INTERPOLATING);
	uint16_t wMoveTime = *pb + (uint16_t)(*(pb+1) << 8);
	uint16_t wMoveIters = (wMoveTime / GETPOSEREGVAL(AX_REG_POSE_FRAME_TIME)) + 1;
	pb +=2;
	uint8_t err = AX_ERROR_NONE;
#ifdef DEBUG_POSE
	abInfoRet[cbInfoRet++] = wMoveIters & 0xff;
	abInfoRet[cbInfoRet++] = wMoveIters >> 8;
#endif		
	// We use only 7 bits per byte as to not get FF values which screw up the format of packets.
	// Since 2 byte registers are stored low to high, should probably have lowest 7 first... High bits in the 5th byte
	// Note: the 5th byte actually only has 3 valid values. So I mask, such that I can encode some options in the other bits.
	uint32_t ulMask = ((uint32_t)(*(pb+4) & 0xf)<<(4*7)) + ((uint32_t)*(pb+3)<<(3*7)) + ((uint32_t)*(pb+2)<<(2*7)) + ((uint32_t)*(pb+1)<<7) + *pb;
	
	// Now lets walk through the data and grab the new position for those slots that have data specified by the bit mask
	pb += 5; //point to the first position value
	nb_bytes -= 7;	//
	for (uint8_t iSlot = 0; ulMask; iSlot++, ulMask >>= 1) {
		if (ulMask & 1) {
			// User told us to use this slot. Make sure we have enough data for it.  Maybe should count number of
			// bits and validate once outside of loop as to keep from inconsistent results...
			if (nb_bytes < 2) {
				err = AX_ERROR_RANGE;
				break;
			}
			g_aPoseinfo[iSlot].next_pose_ = ((uint16_t)(pb[0] + ((uint16_t)(pb[1]) << 8)) << BIOLOID_SHIFT);
			pb += 2;
			nb_bytes -= 2;
			
			// Compute Speed. 
			if (g_aPoseinfo[iSlot].speed_)
				cServosInterpolating--;	// was already moving so don't add twice.
				// Handle case where we already had a speed...
			if (g_aPoseinfo[iSlot].next_pose_ != g_aPoseinfo[iSlot].pose_) {
				if(g_aPoseinfo[iSlot].next_pose_ > g_aPoseinfo[iSlot].pose_) {
					g_aPoseinfo[iSlot].speed_ = (g_aPoseinfo[iSlot].next_pose_ - g_aPoseinfo[iSlot].pose_) / wMoveIters + 1;
				} else {
					g_aPoseinfo[iSlot].speed_ = (g_aPoseinfo[iSlot].pose_ - g_aPoseinfo[iSlot].next_pose_) / wMoveIters + 1;
				}
				cServosInterpolating++;	
			} else {
				g_aPoseinfo[iSlot].speed_ = 0;
			}
#ifdef DEBUG_POSE
			abInfoRet[cbInfoRet++] = iSlot;
			abInfoRet[cbInfoRet++] = GETPOSEREGVAL(iSlot+AX_REG_POSE_ID_FIRST);
			abInfoRet[cbInfoRet++] = g_aPoseinfo[iSlot].pose_ & 0xff;
			abInfoRet[cbInfoRet++] = ((uint16_t)g_aPoseinfo[iSlot].pose_)>>8;
			abInfoRet[cbInfoRet++] = g_aPoseinfo[iSlot].next_pose_ & 0xff;
			abInfoRet[cbInfoRet++] = ((uint16_t)g_aPoseinfo[iSlot].next_pose_)>>8;
			abInfoRet[cbInfoRet++] = g_aPoseinfo[iSlot].speed_ & 0xff;
			abInfoRet[cbInfoRet++] = ((uint16_t)g_aPoseinfo[iSlot].speed_)>>8;
#endif			
		}
	} 
	if (cServosInterpolating < 0)
		cServosInterpolating = 0;

	// If we are now starting a new move 0 out the move timer.
	if (!GETPOSEREGVAL(AX_REG_POSE_INTERPOLATING)) {
		pose_timer = 0;	// timer for processing poses.
		g_wPoseNextFrameTimeout =  GETPOSEREGVAL(AX_REG_POSE_FRAME_TIME)*TIMER_TICS_PER_MS;
		// fudge factor each servo adds 3 bytes (.01 second per byte) so to have commands end at the same time,
		// we fudge our start time to deal with this...
		if (cServosInterpolating)
			g_wPoseNextFrameTimeout += ((uint16_t)(GETPOSEREGVAL(AX_REG_POSE_SIZE) - cServosInterpolating) * 3)/2;
	}

	GETPOSEREGVAL(AX_REG_POSE_INTERPOLATING) = cServosInterpolating;   // don't clear if other move is happening
#ifdef DEBUG_POSE
	abInfoRet[cbInfoRet++] = (uint8_t)cServosInterpolating;
	abInfoRet[cbInfoRet++] = g_abPoseRegVals[AX_REG_POSE_INTERPOLATING-AX_REG_POSE_FIRST_REG];
	abInfoRet[cbInfoRet++] = pose_timer & 0xff;
	abInfoRet[cbInfoRet++] = pose_timer >> 8;
	axStatusPacket(err, abInfoRet, cbInfoRet);
#else
	axStatusPacket(err, 0, 0);
#endif		
}

/*
 * ProcessPoseAbortCmd - This command allows the main processor to tell us they wish for
 * us to abort a current pose at the current state.  With no arguments, will cancel all moves
 * With args, can specify which servos to cancel (pass 2) 
 */
void ProcessPoseAbortCmd(uint8_t nb_bytes, uint8_t *pb) {
	// Pass 1, cancel all, next pass look at PB for servo list to cancel
	for (uint8_t iSlot = 0; iSlot < GETPOSEREGVAL(AX_REG_POSE_SIZE); iSlot++) {
		g_aPoseinfo[iSlot].next_pose_ = g_aPoseinfo[iSlot].pose_ ;
		g_aPoseinfo[iSlot].speed_ = 0;
	}
	GETPOSEREGVAL(AX_REG_POSE_INTERPOLATING) = 0;	// Clear out the Interpolating state.
	axStatusPacket(AX_ERROR_NONE, 0, 0);
}

/* 
 * PoseInterpolateStep - This is the main interpolation function.  We call this from the
 * main loop.  It will first look to see if there are any moves pending and if we have
 * waited long enough from the last time before it will process any changes in the servos
 */

void PoseInterpolateStep(void) {
	// If no interpolation is active or a frame timeout has not happened yet return now. 
	if (!GETPOSEREGVAL(AX_REG_POSE_INTERPOLATING) || (pose_timer < g_wPoseNextFrameTimeout))
		return;
#ifdef DEBUG
	hwbb_up(PORTB7);
#endif
		
	// Don't just set to zero as to not try to accumulate deltas from desired timings.	
	pose_timer = 0;
	
	setTX();	// make sure we are in output mode. 
	int length = 4 + (GETPOSEREGVAL(AX_REG_POSE_INTERPOLATING) * 3);   // 3 = id + pos(2byte)
	int checksum = 254 + length + AX_CMD_SYNC_WRITE + 2 + AX_GOAL_POSITION_L;
	serial_write(0xFF);
	serial_write(0xFF);
	serial_write(0xFE);
	serial_write(length);
	serial_write(AX_CMD_SYNC_WRITE);
	serial_write(AX_GOAL_POSITION_L);
	serial_write(2);

	// Need to loop through all of the slots looking for items that need to be updated.
	// Also build a sync write to do the actual move.
	uint8_t bCntStillMoving = 0;
	for (uint8_t iSlot = 0; iSlot < GETPOSEREGVAL(AX_REG_POSE_SIZE); iSlot++) {
		int diff = (int)g_aPoseinfo[iSlot].next_pose_ - (int)g_aPoseinfo[iSlot].pose_;
		if (diff) {
			if (diff > 0) {
				if (diff <= g_aPoseinfo[iSlot].speed_) {
					g_aPoseinfo[iSlot].pose_ = g_aPoseinfo[iSlot].next_pose_;
					g_aPoseinfo[iSlot].speed_ = 0;
				} else {
					g_aPoseinfo[iSlot].pose_ += g_aPoseinfo[iSlot].speed_;
					bCntStillMoving++;
				}
			} else {
				if (-diff <= g_aPoseinfo[iSlot].speed_) {
					g_aPoseinfo[iSlot].pose_ = g_aPoseinfo[iSlot].next_pose_;
					g_aPoseinfo[iSlot].speed_ = 0;
				} else {
					g_aPoseinfo[iSlot].pose_ -= g_aPoseinfo[iSlot].speed_;
					bCntStillMoving++;
				}
			}

			// Output the three bytes for this servo 
			int temp = g_aPoseinfo[iSlot].pose_ >> BIOLOID_SHIFT;
			checksum += (temp&0xff) + (temp>>8) + GETPOSEREGVAL(iSlot+AX_REG_POSE_ID_FIRST);
			serial_write(GETPOSEREGVAL(iSlot+AX_REG_POSE_ID_FIRST));
			serial_write(temp&0xff);
			serial_write(temp>>8);
		}	
	}	 
	// And output the checksum for the move.
	serial_write(0xff - (checksum % 256));
	GETPOSEREGVAL(AX_REG_POSE_INTERPOLATING) = bCntStillMoving;	// Update cnt to still do...
	g_wPoseNextFrameTimeout =  GETPOSEREGVAL(AX_REG_POSE_FRAME_TIME)*TIMER_TICS_PER_MS;
	// fudge factor each servo adds 3 bytes (.01 second per byte) so to have commands end at the same time, 
	// we fudge our start time to deal with this...
	if (bCntStillMoving)	
		g_wPoseNextFrameTimeout += ((uint16_t)(GETPOSEREGVAL(AX_REG_POSE_SIZE) - bCntStillMoving) * 3)/2;
#ifdef DEBUG
	hwbb_down(PORTB7);
#endif
}

/*
 * ProcessGetVoltageRequest - This procedure returns a voltage value that it retreves from one or more
 *     servos.  When the user has setup a servo to return voltages, our code will periodically ask that
 *     servo(s) for it's current voltage, which we return.  We do this as to be able to try avoid asking at the
 *     wrong time, when we may be active in interpolation. 
 */
void ProcessGetVoltageRequest(void) {
	// BUGBUG:: not implemented yet
	uint8_t bServoID = GETPOSEREGVAL(AX_REG_VOLTAGE);
	if (bServoID == 0xff)
		bServoID = GETPOSEREGVAL(AX_REG_POSE_ID_FIRST);
	passthrough_mode = AX_DIVERT;
	uint8_t bVoltage = axGetRegister(bServoID, AX_REG_PRESENT_VOLTAGE, 1);		
	passthrough_mode = AX_PASSTHROUGH;
	axStatusPacket(AX_ERROR_NONE, &bVoltage, 1);
}
