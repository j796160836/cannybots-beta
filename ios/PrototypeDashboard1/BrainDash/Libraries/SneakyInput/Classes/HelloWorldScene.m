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


//#import "CannybotsController.h"
//#import "CannybotsRacerGlu.h"
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
		
        [self schedule:@selector(tick:) interval:1.0f/20.0f];
        
        [self schedule:@selector(keepAlive:) interval:1.0f];
        
	}
	return self;
    
}
-(void)keepAlive:(float)delta {
    //CannybotsController* cb = [CannybotsController sharedInstance];

    //[cb callMethod:&RACER_PING p1:0];
}

-(void)tick:(float)delta {
    static int lastDir = 0;
    static int lastThrottle= 0;
    int dir      = 255*leftJoystick.velocity.x;
    int throttle = 255*leftJoystick.velocity.y;

#define MSG_LEN 5
    char msg[MSG_LEN] = {0};  // 5 bytes  = 4 bytes data 1 byte NULL terminator
    
    
    // deadzone check
    dir      =      abs(dir) < 10  ? 0 : dir;
    throttle = abs(throttle) < 10  ? 0 : throttle;
    
    //CannybotsController* cb = [CannybotsController sharedInstance];
    BrainSpeakBLE* bsle     = [BrainSpeakBLE sharedInstance];

    
    //if ( (abs(lastDir-dir) > 3) || (abs(lastThrottle-throttle)>3) ){
        //[tb lf_setMotorSpeeds:dir forId1:3 speed2:throttle forId2:4];
        //[cb callMethod:&RACER_JOYAXIS p1:dir p2:throttle];
        uint8_t x =(uint8_t) ((dir+255)>>1);
        uint8_t y =(uint8_t) ((throttle+255)>>1);
        //NSLog(@"dt(%d,%d),  xy=(%d,%d)", dir, throttle, x,y);
        snprintf(msg, MSG_LEN, "%c%c%c%c", 10, x, y, 0 );
        [bsle sendData:[NSData dataWithBytes:msg length:MSG_LEN-1]];
        
        /*
        if ( (dir ==0) && throttle==0) {
            
            [self performBlock:^{
                //[tb lf_setMotorSpeeds:0 forId1:3 speed2:0 forId2:4];
                //[cb callMethod:&RACER_JOYAXIS p1:0 p2:0];
                //snprintf(msg, MSG_LEN, "%c%c%c%c", 10, dir, throttle, 0 );
                //[bsle sendData:[NSData dataWithBytes:msg length:MSG_LEN]];

            } afterDelay: .2];

        }*/
        lastDir = dir;
        lastThrottle= throttle;
    //}

}

@end
