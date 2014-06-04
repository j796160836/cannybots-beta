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
		
        [self schedule:@selector(tick:) interval:1.0f/15.0f];
        
	}
	return self;
    
}

-(void)tick:(float)delta {
    TheBrain* tb = [TheBrain sharedInstance];


    if (rightButton.value) {
        [tb playTone:4 tone:415];
    }
    
    static int16_t leftSpeedLast  = 0;
    static int16_t rightSpeedLast = 0;
    float y = leftJoystick.velocity.y;
    float x = leftJoystick.velocity.x;

    int16_t leftSpeed  = 0;
    int16_t rightSpeed = 0;
    
    float maxSpeed   = 900;
    float deadzone  = 0.1f;
    float pivotZone = 0.2f;

    
    if ( ( fabs(x)<deadzone) && ( fabs(y)< deadzone) ){
        leftSpeed = rightSpeed = 0;
    } else if (x>deadzone) {
        // turn right
        //leftSpeed  = maxSpeed*(y *    x);
        if (fabs(y) < pivotZone) {
            // pivot right
            leftSpeed  = maxSpeed*(x);
            rightSpeed = maxSpeed*(-x);
            
        } else {
            leftSpeed  = maxSpeed*(y);
            rightSpeed = maxSpeed*(y * (1-x));
        }
    } else if (x < -deadzone){
        // turn left
        //rightSpeed = maxSpeed*(y *    fabs(x)  );
        if (fabs(y) < pivotZone) {
            // pivot left
            leftSpeed  = maxSpeed*(x);
            rightSpeed = maxSpeed*(-x);
            
        } else {
            rightSpeed = maxSpeed*(y *    fabs(x)  );
            leftSpeed  = maxSpeed*(y * (1-fabs(x)) );
        }
    } else {
    
        // straight bck/forward
        leftSpeed = rightSpeed = maxSpeed*y;
    }
    
    //NSLog(@"(x,y)=(%f,%f) |  (left, right)=(%d,%d)", x, y, leftSpeed, rightSpeed);
    if ( ( abs(leftSpeed-leftSpeedLast)>5) || ( abs(rightSpeed - rightSpeedLast)>5 )) {
        //Dynamixels:
        //[tb setEndlessTurnMode:YES forId:1];
        //[tb setEndlessTurnMode:YES forId:2];
        //[tb setServoSpeed:leftSpeed forId:1];
        //[tb setServoSpeed:rightSpeed forId:2];
        
        // Line folllowing motors
        [tb lf_setMotorSpeed:leftSpeed forId:1];
        [tb lf_setMotorSpeed:rightSpeed forId:2];
        leftSpeedLast = leftSpeed;
        rightSpeedLast = rightSpeed;
    }
}


@end
