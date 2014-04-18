//
//  BrainDashMainViewController.h
//  BrainDash
//
//  Created by Wayne Keenan on 16/04/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import "BrainDashFlipsideViewController.h"
#import "UARTPeripheral.h"

@interface BrainDashMainViewController : UIViewController <BrainDashFlipsideViewControllerDelegate,UARTPeripheralDelegate, CBCentralManagerDelegate>

typedef enum {
    ConnectionStatusDisconnected = 0,
    ConnectionStatusScanning,
    ConnectionStatusConnected,
} ConnectionStatus;


@property (nonatomic, assign) ConnectionStatus                  connectionStatus;


@end
