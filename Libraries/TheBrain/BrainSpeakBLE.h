//
//  BrainSpeakBLE.h
//  BrainDash
//
//  Created by Wayne Keenan on 21/04/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//


#import <Foundation/Foundation.h>
#import <CoreBluetooth/CBPeripheralManager.h>
#import "UARTPeripheral.h"

#import "CannybotsController.h"


#define BLE_CONN_RETRY_DELAY 1.0f



@interface BrainSpeakBLE : NSObject <UARTPeripheralDelegate, CBCentralManagerDelegate>


typedef enum {
    ConnectionStatusDisconnected = 0,
    ConnectionStatusScanning,
    ConnectionStatusConnected,
} ConnectionStatus;


@property (nonatomic, assign) ConnectionStatus                connectionStatus;

@property (nonatomic, retain) id<CannybotsReceiver>           cbdelegate;


- (void)sendData:(NSData*)newData;

+ (id)sharedInstance;

@end
