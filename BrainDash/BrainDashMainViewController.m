//
//  BrainDashMainViewController.m
//  BrainDash
//
//  Created by Wayne Keenan on 16/04/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import "CBUUID+StringExtraction.h"
#import "BrainDashMainViewController.h"
#import "NSData+hex.h"

@interface BrainDashMainViewController () {

    CBCentralManager    *cm;
    UARTPeripheral      *currentPeripheral;
}
@end

@implementation BrainDashMainViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
	// Do any additional setup after loading the view, typically from a nib.
    cm = [[CBCentralManager alloc] initWithDelegate:self queue:nil];
    
    
    _connectionStatus = ConnectionStatusDisconnected;
    

}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

#pragma mark - Flipside View

- (void)flipsideViewControllerDidFinish:(BrainDashFlipsideViewController *)controller
{
    [self dismissViewControllerAnimated:YES completion:nil];
}

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
    if ([[segue identifier] isEqualToString:@"showAlternate"]) {
        [[segue destinationViewController] setDelegate:self];
    }
}

//////////////////////////////////////////////////////////////////////////////////////
- (IBAction)scanPressed:(id)sender {
    [self scanForPeripherals];
}

- (void)scanForPeripherals{
    
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
    
}




//////////////


- (IBAction)getInfo:(id)sender{
    //Send inputField's string via UART
    //NSString *newString = @"01234567890123456789";
    //NSData *data = [NSData dataWithBytes:newString.UTF8String length:newString.length];
    static const uint8_t magic[] = {
        0xff, 0xff, 0x00, 0x00,
        
        0x01, 0xff, 0x00, 0x00,
        0x01, 0xff, 0x00, 0x00,
        0x01, 0xff, 0x00, 0x00,
        0x01, 0xff, 0x00, 0x00
    };
    NSData *data = [NSData dataWithBytesNoCopy:(void*)magic length:20 freeWhenDone:NO];

    [self sendData:data];
}
#include "NTProtocol.h"

- (IBAction)forward:(id)sender{
    uint8_t msg[NT_MSG_SIZE] = {
        NT_DEFAULT_MSG_HEADER(),
        NT_CREATE_CMD(NT_CAT_AX12, NT_CMD_AX12_SET_ENDLESS_TURN_SPEED, NT_CMD_NO_CONT), 1, bytesFromInt(600),
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP
    };
    NT_MSG_CALC_CRC(msg);
    
    NSData *data = [NSData dataWithBytesNoCopy:(void*)msg length:NT_MSG_SIZE freeWhenDone:NO];
    [self sendData:data];
}
- (IBAction)stop:(id)sender{
    
    uint8_t msg[NT_MSG_SIZE] = {
        NT_DEFAULT_MSG_HEADER(),
        NT_CREATE_CMD(NT_CAT_AX12, NT_CMD_AX12_SET_ENDLESS_TURN_SPEED, NT_CMD_NO_CONT), 1, bytesFromInt(0),
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP
    };
    NT_MSG_CALC_CRC(msg);
    
    NSData *data = [NSData dataWithBytesNoCopy:(void*)msg length:NT_MSG_SIZE freeWhenDone:NO];
    [self sendData:data];
}


- (IBAction)ping:(id)sender{
    uint8_t msg[NT_MSG_SIZE] = {
        NT_DEFAULT_MSG_HEADER(),
        NT_CREATE_CMD(NT_CAT_SONAR, NT_CMD_SONAR_PING, NT_CMD_NO_CONT), 2, bytesFromInt(0),
        NT_CREATE_CMD_NOP,
        //NT_CREATE_CMD(NT_CAT_SONAR, NT_CMD_SONAR_PING, NT_CMD_NO_CONT), 2, bytesFromInt(0),
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP
    };
    NT_MSG_CALC_CRC(msg);
    
    
    NSData *data = [NSData dataWithBytesNoCopy:(void*)msg length:NT_MSG_SIZE freeWhenDone:NO];
    [self sendData:data];
}



- (void)sendData:(NSData*)newData{
    
    //Output data to UART peripheral
    
    NSString *hexString = [newData hexRepresentationWithSpaces:YES];
    NSLog(@"Sending: %@", hexString);
    
    [currentPeripheral writeRawData:newData];
    
}


void dumpCmd(const NT_cmd cmd) {
    NSLog(@"Cat=%d, Cmd=%d, ID=%d, argc=%d, arg1=%d", cmd.category, cmd.command, cmd.id, cmd.argc, cmd.arg1);
}

//Data incoming from BLE UART peripheral

- (void)didReceiveData:(NSData*)newData{
    
    long      len = [newData length];
    uint8_t * buf = (uint8_t*)[newData bytes];

#ifdef NT_DEBUG
    NSString* hexString = [newData hexRepresentationWithSpaces:YES];
    NSLog(@"Received: %@", hexString);
#endif
    
    int status = NT_validateMsg(buf, len);
    
    // Not a valid message? Assume it's debug messages and just log it.
    if (NT_STATUS_OK != status) {
        //NSLog(@"Msg failed msg validation, status code: %c", status);
        NSString* str =   [[NSString alloc] initWithData:newData encoding:NSASCIIStringEncoding];
        NSLog(@"BLE_DBG: %@", str);
        return;
    }

    
    NT_cmd cmd0,cmd1,cmd2,cmd3;
    status = NT_extractCommand(buf, 0, &cmd0);
    status = NT_extractCommand(buf, 1, &cmd1);
    status = NT_extractCommand(buf, 2, &cmd2);
    status = NT_extractCommand(buf, 3, &cmd3);
    dumpCmd(cmd0);
    
}


@end
