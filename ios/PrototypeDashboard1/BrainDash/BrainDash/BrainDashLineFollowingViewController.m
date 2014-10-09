//
//  BrainDashLineFollowingViewController.m
//  BrainDash
//
//  Created by Wayne Keenan on 04/06/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import "BrainDashLineFollowingViewController.h"


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
  
    //NSLog(@"Config Params: %@",configDescriptors);
}

#define PAUSE_HACK   [NSThread sleepForTimeInterval: 0.2];

- (IBAction)goLeft:(id)sender {
    return;
   
}

- (IBAction)goRight:(id)sender {
    return;
    [self goLeft:sender];
    
 }

- (IBAction)stop:(id)sender {
}

- (IBAction)go:(id)sender {
}

- (IBAction)speedChanged:(UISlider*)sender {
    int speed= (int)(sender.value * 255);
    NSLog(@"Speed = %d", speed);
}



@end
