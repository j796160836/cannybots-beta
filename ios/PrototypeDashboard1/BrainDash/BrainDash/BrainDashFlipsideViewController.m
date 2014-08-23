//
//  BrainDashFlipsideViewController.m
//  BrainDash
//
//  Created by Wayne Keenan on 16/04/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

// cocos 2d embedding:  http://www.raywenderlich.com/4817/how-to-integrate-cocos2d-and-uikit

#import "BrainDashFlipsideViewController.h"

@interface BrainDashFlipsideViewController () {
}
@end

@implementation BrainDashFlipsideViewController

- (void) viewWillAppear:(BOOL)animated
{
    [self.navigationController setNavigationBarHidden:YES animated:NO];
    [super viewWillAppear:animated];
}


// Add to end of viewDidUnload

- (void)viewDidLoad
{
    [super viewDidLoad];


}

- (void) viewWillDisappear:(BOOL)animated {
}



- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
}

#pragma mark - Actions

- (IBAction)done:(id)sender
{
    [self.delegate flipsideViewControllerDidFinish:self];
}
- (IBAction)scanPressed:(id)sender {
}


- (IBAction)getInfo:(id)sender{
}

- (IBAction)forward:(id)sender{
}

- (IBAction)backward:(id)sender{
}

- (IBAction)left:(id)sender{
}

- (IBAction)right:(id)sender{
}


- (IBAction)stop:(id)sender{
}


- (IBAction)ping:(id)sender{
}

- (IBAction)playDitty:(id)sender{
}

- (IBAction)soundHorn:(id)sender{
}

- (BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation {
    return YES;
}


@end
