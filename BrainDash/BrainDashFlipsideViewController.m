//
//  BrainDashFlipsideViewController.m
//  BrainDash
//
//  Created by Wayne Keenan on 16/04/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import "BrainDashFlipsideViewController.h"

@interface BrainDashFlipsideViewController () <NTDebugReceiver>{
TheBrain            *theBrain;
}
@end

@implementation BrainDashFlipsideViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    theBrain = [TheBrain sharedInstance];
    
    theBrain.debugDelegate =self;
}

- (void) viewWillDisappear:(BOOL)animated {
    theBrain.debugDelegate = nil;
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    theBrain=nil;
}

#pragma mark - Actions

- (IBAction)done:(id)sender
{
    [self.delegate flipsideViewControllerDidFinish:self];
}


- (void) didReceiveDebug:(NSString*)msg {
    dispatch_async(dispatch_get_main_queue(), ^(void) {
        NSString *newText = [NSString stringWithFormat: @"%@%@\n", _debugTextView.text, msg];
        
        _debugTextView.text = newText;
        
        [_debugTextView scrollRangeToVisible:NSMakeRange(newText.length, 0)];
    });
    //_debugTextView.text = [_debugTextView.text stringByAppendingString:msg];
    //NSLog(@"didReceiveDebug: %@", msg);
}


- (IBAction)scanPressed:(id)sender {
}


- (IBAction)getInfo:(id)sender{
    [theBrain getInfo];
}

- (IBAction)forward:(id)sender{
    [theBrain setServoSpeed:600 forId:1];
    [theBrain setServoSpeed:600 forId:2];
}

- (IBAction)stop:(id)sender{
    [theBrain setServoSpeed:0 forId:1];
    [theBrain setServoSpeed:0 forId:2];
}


- (IBAction)ping:(id)sender{
    [theBrain ping:1];
}




@end
