//
//  BrainSpeakBLE.m
//  BrainDash
//
//  Created by Wayne Keenan on 21/04/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import "BrainSpeakBLE.h"

#import "CBUUID+StringExtraction.h"
#import "NSData+hex.h"

@interface BrainSpeakBLE()

@property CBCentralManager    *cm;
@property UARTPeripheral      *currentPeripheral;
@end


@implementation BrainSpeakBLE



#pragma mark Singleton Methods

+ (id)sharedInstance {
    static BrainSpeakBLE *sharedBrainSpeakBLE = nil;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        sharedBrainSpeakBLE = [[self alloc] init];
    });
    return sharedBrainSpeakBLE;
}

- (id)init {
    if (self = [super init]) {
        //someProperty = [[NSString alloc] initWithString:@"Default Property Value"];
        
        _cm = [[CBCentralManager alloc] initWithDelegate:self queue:nil];
        _connectionStatus = ConnectionStatusDisconnected;

        [self reconnectBLE];
        

    }
    return self;
}

- (void)dealloc {
    // Should never be called, but just here for clarity really.
}




//////////////////////////////////////////////////////////////////////////////////////


- (void)scanForPeripherals{
    NSLog(@"scanForPeripherals");
    if (_connectionStatus == ConnectionStatusConnected)
        return;
    
    //Look for available Bluetooth LE devices
    _connectionStatus = ConnectionStatusScanning;
    
    
    //skip scanning if UART is already connected
    NSArray *connectedPeripherals = [_cm retrieveConnectedPeripheralsWithServices:@[UARTPeripheral.uartServiceUUID]];
    if ([connectedPeripherals count] > 0) {
        //connect to first peripheral in array
        [self connectPeripheral:[connectedPeripherals objectAtIndex:0]];
    }
    
    else{
        
        [_cm scanForPeripheralsWithServices:@[UARTPeripheral.uartServiceUUID]
                                   options:@{CBCentralManagerScanOptionAllowDuplicatesKey: [NSNumber numberWithBool:NO]}];
    }
    
}


- (void)connectPeripheral:(CBPeripheral*)peripheral{
    NSLog(@"connectPeripheral");

    //Connect Bluetooth LE device
    
    //Clear off any pending connections
    [_cm cancelPeripheralConnection:peripheral];
    
    //Connect
    _currentPeripheral = [[UARTPeripheral alloc] initWithPeripheral:peripheral delegate:self];
    [_cm connectPeripheral:peripheral options:@{CBConnectPeripheralOptionNotifyOnDisconnectionKey: [NSNumber numberWithBool:YES]}];
    
}


- (void)disconnect{
    NSLog(@"disconnect");

    //Disconnect Bluetooth LE device
    
    _connectionStatus = ConnectionStatusDisconnected;
    
    [_cm cancelPeripheralConnection:_currentPeripheral.peripheral];
    [self reconnectBLE];
}



#pragma mark CBCentralManagerDelegate


- (void) centralManagerDidUpdateState:(CBCentralManager*)central{
    NSLog(@"centralManagerDidUpdateState: %lx", central.state);

    if (central.state == CBCentralManagerStatePoweredOn){
        
        //respond to powered on
        [self reconnectBLE];

    }
    
    else if (central.state == CBCentralManagerStatePoweredOff){
        
        //respond to powered off
    }
    
}


- (void) centralManager:(CBCentralManager*)central didDiscoverPeripheral:(CBPeripheral*)peripheral advertisementData:(NSDictionary*)advertisementData RSSI:(NSNumber*)RSSI{
    
    NSLog(@"Did discover peripheral %@  UUID=%@", peripheral.name, peripheral.UUID);
    
    NSLog(@"!!!!!!!!!!!!!! change this fucntion to get ALL devices .... BrainSpeakBLE.didDiscoverPeripheral");
    [_cm stopScan];
    
    [self connectPeripheral:peripheral];
}


- (void) centralManager:(CBCentralManager*)central didConnectPeripheral:(CBPeripheral*)peripheral{
    
    NSLog(@"didConnectPeripheral");

    if ([_currentPeripheral.peripheral isEqual:peripheral]){
        
        if(peripheral.services){
            NSLog(@"Did connect to existing peripheral %@", peripheral.name);
            [_currentPeripheral peripheral:peripheral didDiscoverServices:nil]; //already discovered services, DO NOT re-discover. Just pass along the peripheral.
        }
        
        else{
            NSLog(@"Did connect peripheral %@", peripheral.name);
            [_currentPeripheral didConnect];
        }
    }
}


- (void) centralManager:(CBCentralManager*)central didDisconnectPeripheral:(CBPeripheral*)peripheral error:(NSError*)error{
    
    NSLog(@"Did disconnect peripheral %@", peripheral.name);
    
    //respond to disconnected
    [self peripheralDidDisconnect];
    
    if ([_currentPeripheral.peripheral isEqual:peripheral])
    {
        [_currentPeripheral didDisconnect];
    }
}


#pragma mark UARTPeripheralDelegate


- (void)didReadHardwareRevisionString:(NSString*)string{
    
    //Once hardware revision string is read, connection to Bluefruit is complete
    
    NSLog(@"HW Revision: %@", string);
    
    _connectionStatus = ConnectionStatusConnected;
    
    
    
}


- (void)uartDidEncounterError:(NSString*)error{
    
    
    NSLog(@"%@", error);
    
    //[alert show];
    [self performSelector:@selector(scanForPeripherals) withObject:nil afterDelay:BLE_CONN_RETRY_DELAY];
    
}




- (void)peripheralDidDisconnect{
    
    //respond to device disconnecting
    
    [self uartDidEncounterError:@"Peripheral disconnected"];
    
    //if status was connected, then disconnect was unexpected by the user, show alert
    if (_connectionStatus == ConnectionStatusConnected) {
        //display disconnect alert
        UIAlertView *alert = [[UIAlertView alloc]initWithTitle:@"Disconnected"
                                                       message:@"BLE peripheral has disconnected"
                                                      delegate:nil
                                             cancelButtonTitle:@"OK"
                                             otherButtonTitles: nil];
        
        NSLog(@"%@", alert);
        
        //[alert show];
    }
    
    _connectionStatus = ConnectionStatusDisconnected;
    [self reconnectBLE];
}


- (void)alertBluetoothPowerOff{
    
    //Respond to system's bluetooth disabled
    
    NSString *message   = @"You must turn on Bluetooth in Settings in order to connect to a device";
    NSLog(@"%@", message);
}


- (void)alertFailedConnection{
    
    //Respond to unsuccessful connection
    NSString *message   = @"Please check power & wiring,\nthen reset your Arduino";
    NSLog(@"%@", message);
    //[alertView show];
    
    [self reconnectBLE];
    
}





- (void) reconnectBLE {
    [self performSelector:@selector(scanForPeripherals) withObject:nil afterDelay:BLE_CONN_RETRY_DELAY];
}



// Delegate called by 'TheBrain' to get data to the device
- (void)sendData:(NSData*)newData{
    //Output data to UART peripheral
    NSString *hexString = [newData hexRepresentationWithSpaces:YES];
    NSLog(@"Sending: %@", hexString);
    [_currentPeripheral writeRawData:newData];
}

// called by the UART
- (void) didReceiveData:(NSData*)newData{
    [self.cbdelegate didReceiveData:newData];
    
}


@end
