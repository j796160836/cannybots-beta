//
//  BrainDashLineFollowingViewController.m
//  BrainDash
//
//  Created by Wayne Keenan on 04/06/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import "BrainDashLineFollowingViewController.h"
#import "TheBrain.h"
#import "CannybotsLineFollowing.h"

@interface BrainDashLineFollowingViewController ()
{
    TheBrain            *theBrain;
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
    theBrain = [TheBrain sharedInstance];
    
}

- (void) viewDidAppear:(BOOL)animated {
    theBrain.appDelegate = self;
}

- (void) viewWillDisappear:(BOOL)animated {
    theBrain.appDelegate = nil;
    
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
    [theBrain lf_switchNextJunction];
}

- (IBAction)goLeft:(id)sender {
    [theBrain lf_left];
}
- (IBAction)goRight:(id)sender {
    [theBrain lf_right];
}
- (IBAction)stop:(id)sender {
    [theBrain lf_stop];

}
- (IBAction)go:(id)sender {
    [theBrain lf_go];


}
- (IBAction)speedChanged:(UISlider*)sender {
    int speed= (int)(sender.value * 255);
    NSLog(@"Speed = %d", speed);
    [theBrain lf_speed:speed];
}



- (void) didReceiveCommand:(uint8_t)cmd forId:(uint8_t)_id withArg:(int16_t)arg1 {
    
    NSLog(@"CMD=%d, id=%d, arg=%d", cmd, _id, arg1);
    if (NT_CMD_LINEFOLLOW_MOVE == cmd) {
        if (LINEFOLLOW_STOP == _id){
             [self.tabBarController setSelectedIndex:1];
        } else {
            NSLog(@"WARNING: unrecognised CMD value: %d",_id);
        }
    }   
}


@end
