//
//  BrainDashMainViewController.h
//  BrainDash
//
//  Created by Wayne Keenan on 16/04/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//


#import "TheBrain.h"
#import "HelloWorldScene.h"
#import <cocos2d.h>

@interface BrainDashMainViewController : UIViewController <BrainDashFlipsideViewControllerDelegate, CCDirectorDelegate>

@property (weak, nonatomic) IBOutlet UIView *joypadView;


@end
