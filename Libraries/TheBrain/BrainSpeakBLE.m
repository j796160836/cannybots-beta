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


@implementation BrainSpeakBLE

CBCentralManager    *cm;
UARTPeripheral      *currentPeripheral;


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
        
        cm = [[CBCentralManager alloc] initWithDelegate:self queue:nil];
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
    
    if (_connectionStatus == ConnectionStatusConnected)
        return;
    
    //Look for available Bluetooth LE devices
    _connectionStatus = ConnectionStatusScanning;
    
    
    //skip scanning if UART is already connected
    NSArray *connectedPeripherals = [cm retrieveConnectedPeripheralsWithServices:@[UARTPeripheral.uartServiceUUID]];
    if ([connectedPeripherals count] > 0) {
        //connect to first peripheral in array
        [self connectPeripheral:[connectedPeripherals objectAtIndex:0]];
    }
    
    else{
        
        [cm scanForPeripheralsWithServices:@[UARTPeripheral.uartServiceUUID]
                                   options:@{CBCentralManagerScanOptionAllowDuplicatesKey: [NSNumber numberWithBool:NO]}];
    }
    
}


- (void)connectPeripheral:(CBPeripheral*)peripheral{
    
    //Connect Bluetooth LE device
    
    //Clear off any pending connections
    [cm cancelPeripheralConnection:peripheral];
    
    //Connect
    currentPeripheral = [[UARTPeripheral alloc] initWithPeripheral:peripheral delegate:self];
    [cm connectPeripheral:peripheral options:@{CBConnectPeripheralOptionNotifyOnDisconnectionKey: [NSNumber numberWithBool:YES]}];
    
}


- (void)disconnect{
    
    //Disconnect Bluetooth LE device
    
    _connectionStatus = ConnectionStatusDisconnected;
    
    [cm cancelPeripheralConnection:currentPeripheral.peripheral];
    [self reconnectBLE];
}



#pragma mark CBCentralManagerDelegate


- (void) centralManagerDidUpdateState:(CBCentralManager*)central{
    
    if (central.state == CBCentralManagerStatePoweredOn){
        
        //respond to powered on
    }
    
    else if (central.state == CBCentralManagerStatePoweredOff){
        
        //respond to powered off
    }
    
}


- (void) centralManager:(CBCentralManager*)central didDiscoverPeripheral:(CBPeripheral*)peripheral advertisementData:(NSDictionary*)advertisementData RSSI:(NSNumber*)RSSI{
    
    NSLog(@"Did discover peripheral %@", peripheral.name);
    
    [cm stopScan];
    
    [self connectPeripheral:peripheral];
}


- (void) centralManager:(CBCentralManager*)central didConnectPeripheral:(CBPeripheral*)peripheral{
    
    if ([currentPeripheral.peripheral isEqual:peripheral]){
        
        if(peripheral.services){
            NSLog(@"Did connect to existing peripheral %@", peripheral.name);
            [currentPeripheral peripheral:peripheral didDiscoverServices:nil]; //already discovered services, DO NOT re-discover. Just pass along the peripheral.
        }
        
        else{
            NSLog(@"Did connect peripheral %@", peripheral.name);
            [currentPeripheral didConnect];
        }
    }
}


- (void) centralManager:(CBCentralManager*)central didDisconnectPeripheral:(CBPeripheral*)peripheral error:(NSError*)error{
    
    NSLog(@"Did disconnect peripheral %@", peripheral.name);
    
    //respond to disconnected
    [self peripheralDidDisconnect];
    
    if ([currentPeripheral.peripheral isEqual:peripheral])
    {
        [currentPeripheral didDisconnect];
    }
}


#pragma mark UARTPeripheralDelegate


- (void)didReadHardwareRevisionString:(NSString*)string{
    
    //Once hardware revision string is read, connection to Bluefruit is complete
    
    NSLog(@"HW Revision: %@", string);
    
    _connectionStatus = ConnectionStatusConnected;
    
    
    
}


- (void)uartDidEncounterError:(NSString*)error{
    
    //Display error alert
    UIAlertView *alert = [[UIAlertView alloc]initWithTitle:@"Error"
                                                   message:error
                                                  delegate:nil
                                         cancelButtonTitle:@"OK"
                                         otherButtonTitles:nil];
    
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
    
    NSString *title     = @"Bluetooth Power";
    NSString *message   = @"You must turn on Bluetooth in Settings in order to connect to a device";
    UIAlertView *alertView = [[UIAlertView alloc] initWithTitle:title message:message delegate:nil cancelButtonTitle:@"OK" otherButtonTitles:nil];
    NSLog(@"%@", message);
    
    //[alertView show];
}


- (void)alertFailedConnection{
    
    //Respond to unsuccessful connection
    
    NSString *title     = @"Unable to connect";
    NSString *message   = @"Please check power & wiring,\nthen reset your Arduino";
    UIAlertView *alertView = [[UIAlertView alloc] initWithTitle:title message:message delegate:nil cancelButtonTitle:@"OK" otherButtonTitles:nil];
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
    [currentPeripheral writeRawData:newData];
}

// called by the UART
- (void) didReceiveData:(NSData*)newData{
    [self.delegate didReceiveData:newData];
    
}



@end
