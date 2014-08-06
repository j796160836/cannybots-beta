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


#import "CannybotsController.h"
#import "CannybotsRacer.h"

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
        
		leftJoy.joystick = [[SneakyJoystick alloc] initWithRect:leftJoy.backgroundSprite.boundingBox];
		leftJoystick = leftJoy.joystick;
		[self addChild:leftJoy];
		
        
  /*
		SneakyButtonSkinnedBase *rightBut = [[SneakyButtonSkinnedBase alloc] init];
		rightBut.position = ccp(256,32*5);
//        rightBut.defaultSprite = [ColoredCircleSprite circleWithColor:[CCColor colorWithRed:0.5 green:1 blue:0.5 alpha:0.5f] radius:32];
  //      rightBut.activatedSprite = [ColoredCircleSprite circleWithColor:[CCColor colorWithRed:1 green:0 blue:0 alpha:1] radius:32];
    //    rightBut.pressSprite = [ColoredCircleSprite circleWithColor:[CCColor colorWithRed:1 green:0 blue:0 alpha:1] radius:32];
//		rightBut.button = [[SneakyButton alloc] initWithRect:CGRectMake(0, 0, 64, 64)];
		rightButton = rightBut.button;
		rightButton.isToggleable = YES;
        //rightButton.isHoldable = YES;
		//[self addChild:rightBut];
	*/
        
		[[CCDirector sharedDirector] setAnimationInterval:1.0f/60.0f];
		
        [self schedule:@selector(tick:) interval:1.0f/15.0f];
        
        //[self schedule:@selector(keepAlive:) interval:1.0f];
        
	}
	return self;
    
}
-(void)keepAlive:(float)delta {
    CannybotsController* cb = [CannybotsController sharedInstance];

    [cb callMethod:&RACER_PING p1:0];
}

-(void)tick:(float)delta {
    static int lastDir = 0;
    static int lastThrottle= 0;
    int dir      = 255*leftJoystick.velocity.x;
    int throttle = 255*leftJoystick.velocity.y;

    // deadzone check
    dir      =      abs(dir) < 10  ? 0 : dir;
    throttle = abs(throttle) < 10  ? 0 : throttle;
    
    CannybotsController* cb = [CannybotsController sharedInstance];
    
    if ( (abs(lastDir-dir) > 3) || (abs(lastThrottle-throttle)>3) ){
        //[tb lf_setMotorSpeeds:dir forId1:3 speed2:throttle forId2:4];
        [cb callMethod:&RACER_JOYAXIS p1:dir p2:throttle];

        if ( (dir ==0) && throttle==0) {
            
            [self performBlock:^{
                //[tb lf_setMotorSpeeds:0 forId1:3 speed2:0 forId2:4];
                [cb callMethod:&RACER_JOYAXIS p1:0 p2:0];

            } afterDelay: .2];

        }
        lastDir = dir;
        lastThrottle= throttle;
    }

}

@end
