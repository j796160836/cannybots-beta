//
//  HelloWorldLayer.h
//  SneakyInput 0.4.0
//
//  Created by Nick Pannuto on 12/3/10.
//  Copyright Sneakyness, llc. 2010. All rights reserved.
//

#import "cocos2d.h"

@class SneakyJoystick;
@class SneakyButton;

@interface HelloWorld : CCScene
{
	SneakyJoystick *leftJoystick;
	SneakyJoystick *rightJoystick;
	SneakyButton *rightButton;
}


+ (HelloWorld *)scene;
-(id) init;
@end
