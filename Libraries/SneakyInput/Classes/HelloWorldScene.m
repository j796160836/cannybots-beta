//
//  HelloWorldLayer.m
//  SneakyInput 0.4.0
//
//  Created by Nick Pannuto on 12/3/10.
//  Copyright Sneakyness, llc. 2010. All rights reserved.
//

#import "HelloWorldScene.h"
#import "SneakyJoystick.h"
#import "SneakyJoystickSkinnedJoystickExample.h"
#import "SneakyJoystickSkinnedDPadExample.h"
#import "SneakyButton.h"
#import "SneakyButtonSkinnedBase.h"
#import "ColoredCircleSprite.h"
#import "CannybotsLineFollowing.h"

#import "TheBrain.h"

@implementation HelloWorld

+ (HelloWorld *)scene;
{
	return [[self alloc] init];
}


-(id) init
{
	if( (self=[super init] )) {
        
        CCNodeColor *background =[CCNodeColor nodeWithColor:[CCColor colorWithRed:1.0f green:1.0f blue:1.0f alpha:1.0f]];
        [self addChild:background];
        
        int w = [[CCDirector sharedDirector] viewSize].width;
        int h = [[CCDirector sharedDirector] viewSize].height;
        
		SneakyJoystickSkinnedBase *leftJoy = [[SneakyJoystickSkinnedBase alloc] init];
		leftJoy.position = ccp(w/2,h/2);
        leftJoy.backgroundSprite = [CCSprite spriteWithImageNamed:@"joystick.png"];

        
        leftJoy.thumbSprite =  [CCSprite spriteWithImageNamed:@"knob.png"];
		leftJoy.joystick = [[SneakyJoystick alloc] initWithRect:CGRectMake(0,0,180,180)];
		leftJoystick = leftJoy.joystick;
		[self addChild:leftJoy];
		
        
        
		SneakyButtonSkinnedBase *rightBut = [[SneakyButtonSkinnedBase alloc] init];
		rightBut.position = ccp(256,32*5);
        rightBut.defaultSprite = [ColoredCircleSprite circleWithColor:[CCColor colorWithRed:0.5 green:1 blue:0.5 alpha:0.5f] radius:32];
        rightBut.activatedSprite = [ColoredCircleSprite circleWithColor:[CCColor colorWithRed:1 green:0 blue:0 alpha:1] radius:32];
        rightBut.pressSprite = [ColoredCircleSprite circleWithColor:[CCColor colorWithRed:1 green:0 blue:0 alpha:1] radius:32];
		rightBut.button = [[SneakyButton alloc] initWithRect:CGRectMake(0, 0, 64, 64)];
		rightButton = rightBut.button;
		rightButton.isToggleable = YES;
        //rightButton.isHoldable = YES;
		//[self addChild:rightBut];
		
		[[CCDirector sharedDirector] setAnimationInterval:1.0f/60.0f];
		
        [self schedule:@selector(tick:) interval:1.0f/5.0f];
        
	}
	return self;
    
}

-(void)tick:(float)delta {
    TheBrain* tb = [TheBrain sharedInstance];


    if (rightButton.value) {
        [tb playTone:4 tone:415];
    }
    
    float y = leftJoystick.velocity.y;
    float x = leftJoystick.velocity.x;

    int dir = MOTOR_MAX_SPEED*x;
    int throttle =MOTOR_MAX_SPEED*y;
    
    //NSLog(@"%d,%d", dir, throttle);
    static int lastDir = 0;
    static int lastThrottle= 0;

    // deadzone check (save radio tx)
    if ( (abs(dir) < 10) || (abs(throttle)<10)){
        dir=0;
        throttle=0;
        if (lastDir != 0  || lastThrottle!=0) {
            [tb lf_setMotorSpeed:dir forId:3];
            [tb lf_setMotorSpeed:throttle forId:4];
            [tb lf_setMotorSpeed:dir forId:3];
            [tb lf_setMotorSpeed:throttle forId:4];
            [tb lf_setMotorSpeed:dir forId:3];
            [tb lf_setMotorSpeed:throttle forId:4];
        }
        lastDir = dir;
        lastThrottle= throttle;
    }

    if ( (abs(lastDir-dir) > 0) || (abs(lastThrottle-throttle)>0) ){
        
        [tb lf_setMotorSpeed:dir forId:3];
        [tb lf_setMotorSpeed:throttle forId:4];
        lastDir = dir;
        lastThrottle= throttle;
        
    }
}


@end
