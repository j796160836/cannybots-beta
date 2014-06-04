//
//  BrainDashLineFollowingViewController.m
//  BrainDash
//
//  Created by Wayne Keenan on 04/06/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import "BrainDashLineFollowingViewController.h"
#import "TheBrain.h"

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
    [theBrain lf_speed:sender.value * 1024];
}
@end
