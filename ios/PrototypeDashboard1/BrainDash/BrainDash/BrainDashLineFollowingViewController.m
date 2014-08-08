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
}
*/
- (IBAction)swapLanes:(id)sender {
    CannybotsController* cb = [CannybotsController sharedInstance];
    
    NSArray* configDescriptors = [cb getConfigParameterList];

    [cb setConfigParameter_INT16:&cfg_ir_max   p1:1000];
    [cb setConfigParameter_INT16:&cfg_ir_whiteThreshold   p1:700];
    [cb setConfigParameter_UINT8:&cfg_ir_pin_1 p1:24];
    [cb setConfigParameter_UINT8:&cfg_ir_pin_2 p1:26];
    [cb setConfigParameter_UINT8:&cfg_ir_pin_3 p1:29];

    NSLog(@"Config Params: %@",configDescriptors);
}

- (IBAction)goLeft:(id)sender {

    CannybotsController* cb = [CannybotsController sharedInstance];

    [cb setConfigParameter_INT16:&cfg_ir_max   p1:1000];
    [cb setConfigParameter_INT16:&cfg_ir_whiteThreshold   p1:700];
    // TODO: cratea convineice for Arduiono pins
    [cb setConfigParameter_UINT8:&cfg_ir_pin_1 p1:24];  // A6
    [cb setConfigParameter_UINT8:&cfg_ir_pin_2 p1:26];  // A8
    [cb setConfigParameter_UINT8:&cfg_ir_pin_3 p1:29];  // A10

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
