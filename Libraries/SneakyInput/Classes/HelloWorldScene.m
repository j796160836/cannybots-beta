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
        
        // Create a colored background (Dark Grey)
        //CCNodeColor *background = [[CCNodeColor ]]//[CCNodeColor nodeWithColor:[CCColor colorWithRed:0.2f green:0.2f blue:0.2f alpha:0.0f]];
        //[self addChild:background];

		
		SneakyJoystickSkinnedBase *leftJoy = [[SneakyJoystickSkinnedBase alloc] init];
		leftJoy.position = ccp(64*2,64*2);
        leftJoy.backgroundSprite = [ColoredCircleSprite circleWithColor:[CCColor colorWithRed:1 green:0 blue:0 alpha:0.5] radius:64];
        leftJoy.thumbSprite = [ColoredCircleSprite circleWithColor:[CCColor colorWithRed:0 green:0 blue:1 alpha:0.8f] radius:32];
		leftJoy.joystick = [[SneakyJoystick alloc] initWithRect:CGRectMake(0,0,128,128)];
		leftJoystick = leftJoy.joystick;
		[self addChild:leftJoy];
		
		SneakyButtonSkinnedBase *rightBut = [[SneakyButtonSkinnedBase alloc] init];
		rightBut.position = ccp(256,32*5);
        rightBut.defaultSprite = [ColoredCircleSprite circleWithColor:[CCColor colorWithRed:0.5 green:1 blue:0.5 alpha:0.5f] radius:32];
        rightBut.activatedSprite = [ColoredCircleSprite circleWithColor:[CCColor colorWithRed:1 green:1 blue:1 alpha:1] radius:32];
        rightBut.pressSprite = [ColoredCircleSprite circleWithColor:[CCColor colorWithRed:1 green:0 blue:0 alpha:1] radius:32];
		rightBut.button = [[SneakyButton alloc] initWithRect:CGRectMake(0, 0, 64, 64)];
		rightButton = rightBut.button;
		rightButton.isToggleable = YES;
        //rightButton.isHoldable = YES;
		[self addChild:rightBut];
		
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


    if ( ( fabs(x)<deadzone) && ( fabs(y)< deadzone) ){
        leftSpeed = rightSpeed = 0;
    } else if (x>deadzone) {
        // turn right
        //leftSpeed  = maxSpeed*(y *    x);
        leftSpeed  = maxSpeed*(y);
        rightSpeed = maxSpeed*(y * (1-x)); //TODO: support spinning on the spot.. make speed go negative when x passes 0.5
    } else if (x < -deadzone){
        // turn left
        //rightSpeed = maxSpeed*(y *    fabs(x)  );
        rightSpeed = maxSpeed*(y *    fabs(x)  );
        leftSpeed  = maxSpeed*(y * (1-fabs(x)) ); //TODO: support spinning on the spot.. make speed go negative when x passes 0.5

    } else {
    
        // straight
        leftSpeed = rightSpeed = maxSpeed*y;
    }
    
    //NSLog(@"(x,y)=(%f,%f) |  (left, right)=(%d,%d)", x, y, leftSpeed, rightSpeed);
    if ( ( abs(leftSpeed-leftSpeedLast)>5) || ( abs(rightSpeed - rightSpeedLast)>5 )) {
                [tb setServoSpeed:leftSpeed forId:1];
        [tb setServoSpeed:rightSpeed forId:2];
        leftSpeedLast = leftSpeed;
        rightSpeedLast = rightSpeed;
    }
}


@end
