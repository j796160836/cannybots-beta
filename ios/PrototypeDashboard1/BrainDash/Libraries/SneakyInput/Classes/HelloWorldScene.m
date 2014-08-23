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
#import "NSObject+performBlockAfterDelay.h"

#import "BrainSpeakBLE.h"
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
		leftJoy.position = ccp(w/4,h/2);
        leftJoy.backgroundSprite = [CCSprite spriteWithImageNamed:@"joystick.png"];

        
        leftJoy.thumbSprite =  [CCSprite spriteWithImageNamed:@"knob.png"];
        
		leftJoy.joystick = [[SneakyJoystick alloc] initWithRect:leftJoy.backgroundSprite.boundingBox];
		leftJoystick = leftJoy.joystick;
		[self addChild:leftJoy];
		
        
  
		SneakyButtonSkinnedBase *rightBut = [[SneakyButtonSkinnedBase alloc] init];
		rightBut.position = ccp(w- w/8  ,h/2);
        rightBut.defaultSprite = [CCSprite spriteWithImageNamed:@"button.png"];
        rightBut.activatedSprite = [CCSprite spriteWithImageNamed:@"button.png"];
        rightBut.pressSprite = [CCSprite spriteWithImageNamed:@"button.png"];

		rightBut.button = [[SneakyButton alloc] initWithRect:CGRectMake(0, 0, 64, 64)];
		rightButton = rightBut.button;
		rightButton.isToggleable = YES;
        rightButton.isHoldable = YES;
		[self addChild:rightBut];
        
        
		[[CCDirector sharedDirector] setAnimationInterval:1.0f/25.0f];
		
        [self schedule:@selector(tick:) interval:1.0f/10.0f];
        
        //[self schedule:@selector(keepAlive:) interval:0.20f];
        
	}
	return self;
    
}

int lastDir = 0;
int lastThrottle= 0;
int dir      = 0;
int throttle = 0;

-(void)keepAlive:(float)delta {
}


long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


-(void)tick:(float)delta {
    dir      = 255*leftJoystick.velocity.x;
    throttle = 255*leftJoystick.velocity.y;
    // deadzone check
    dir      =      abs(dir) < 10  ? 0 : dir;
    throttle = abs(throttle) < 10  ? 0 : throttle;
    // CannybotsController* cb = [CannybotsController sharedInstance];
    char msg[21] = {0};
    snprintf(msg, sizeof(msg), "%c%5.5s%c%c%c%c%c%c",1, "JOY01", highByte(dir), lowByte(dir), highByte(throttle), lowByte(throttle), highByte(0), lowByte(0));
    //snprintf(msg, sizeof(msg), "%c%c%c%c",0, map(dir, -255,255,0,255), map(throttle, -255,255, 0, 255), rightButton.active == YES);
    NSData *data = [NSData dataWithBytesNoCopy:msg length:20 freeWhenDone:NO];
    BrainSpeakBLE*  bsle = [BrainSpeakBLE sharedInstance];
    [bsle sendData:data];
    
    //[cb writeInt:dir forVariable:@"JOY_X"];
    //[cb writeInt:throttle forVariable:@"JOY_Y"];
}

@end

