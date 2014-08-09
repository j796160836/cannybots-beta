//
//  BrainDashLineFollowingViewController.m
//  BrainDash
//
//  Created by Wayne Keenan on 04/06/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import "BrainDashLineFollowingViewController.h"
#import <CannybotsController.h>

#import "CannybotsRacer.h"

@interface BrainDashLineFollowingViewController ()
{
}
@end

@implementation BrainDashLineFollowingViewController

- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (self) {
        // Custom initialization
    }
    return self;
}

- (void)viewDidLoad
{
    [super viewDidLoad];
}

- (void) viewDidAppear:(BOOL)animated {
}

- (void) viewWillDisappear:(BOOL)animated {
}


- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

/*
#pragma mark - Navigation

// In a storyboard-based application, you will often want to do a little preparation before navigation
- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
    // Get the new view controller using [segue destinationViewController].
    // Pass the selected object to the new view controller.
}3
*/
- (IBAction)swapLanes:(id)sender {
    CannybotsController* cb = [CannybotsController sharedInstance];
    
    //NSArray* configDescriptors =
    [cb getConfigParameterList]; // result returned async

    //NSLog(@"Config Params: %@",configDescriptors);
}

#define PAUSE_HACK   [NSThread sleepForTimeInterval: 0.5];

- (IBAction)goLeft:(id)sender {

    CannybotsController* cb = [CannybotsController sharedInstance];

    [cb setConfigParameter_UINT8:&cfg_battery_hasSense p1:0];
    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_battery_pin_sense p1:0];
    PAUSE_HACK;
    
    // TODO: crate convineice for Arduiono pins
    [cb setConfigParameter_UINT8:&cfg_ir_pin_1 p1:24];  // A6
    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_ir_pin_2 p1:26];  // A8
    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_ir_pin_3 p1:29];  // A10
    PAUSE_HACK;


    [cb setConfigParameter_UINT8:&cfg_motorDriver_maxSpeed p1:255];
    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorDriver_hasDriveMode p1:1];
    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorDriver_hasMotorSense p1:0];
    PAUSE_HACK;

    [cb setConfigParameter_UINT8:&cfg_motorDriver_driveModePin p1:2]; // 255 for max motor speed fix
    //TODO: fixstruct offset issue, (need to conver 'local' offset to positianl value and send that to be converted back by the bot
    PAUSE_HACK;

    [cb setConfigParameter_UINT8:&cfg_motorA_id p1:0];
    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorA_postiveSpeedisFwd p1:1];
    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorA_pin_1 p1:3];
    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorA_pin_2 p1:5];
    PAUSE_HACK;

    [cb setConfigParameter_UINT8:&cfg_motorB_id p1:1];
    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorB_postiveSpeedisFwd p1:0];
    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorB_pin_1 p1:6];
    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorB_pin_2 p1:9];
    PAUSE_HACK;

    [cb setConfigParameter_UINT8:&cfg_cruiseSpeed_defaultSpeed p1:120];
    PAUSE_HACK;

    [cb setConfigParameter_UINT8:&cfg_motor_speedSmoothingDivisions p1:1];
    PAUSE_HACK;
    
    [cb setConfigParameter_UINT8:&cfg_motor_speedSmoothingMaxDelta p1:255];
    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_joystick_xAxisDeadzone p1:50];
    PAUSE_HACK;

    [cb setConfigParameter_INT16:&cfg_ir_max   p1:1000];
    PAUSE_HACK;
    [cb setConfigParameter_INT16:&cfg_ir_whiteThreshold   p1:700];
    PAUSE_HACK;

    
    [cb setConfigParameter_INT16:&cfg_offLineMaxTime   p1:200];
    PAUSE_HACK;
    [cb setConfigParameter_UINT16:&cfg_info_printValsInterval p1:1000];
    PAUSE_HACK;

}
- (IBAction)goRight:(id)sender {

}
- (IBAction)stop:(id)sender {
}
- (IBAction)go:(id)sender {

}
- (IBAction)speedChanged:(UISlider*)sender {
    int speed= (int)(sender.value * 255);
    NSLog(@"Speed = %d", speed);
}



- (void) didReceiveCommand:(uint8_t)cmd forId:(uint8_t)_id withArg:(int16_t)arg1 {
//             [self.tabBarController setSelectedIndex:1];
}


@end
