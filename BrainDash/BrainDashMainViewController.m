//
//  BrainDashMainViewController.m
//  BrainDash
//
//  Created by Wayne Keenan on 16/04/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import "BrainDashMainViewController.h"



@interface BrainDashMainViewController () {
    TheBrain            *theBrain;
}
@end

@implementation BrainDashMainViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    theBrain = [TheBrain sharedInstance];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
    theBrain=nil;
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






@end
