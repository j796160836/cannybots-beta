//
//  BrainDashMainViewController.m
//  BrainDash
//
//  Created by Wayne Keenan on 16/04/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import "BrainDashMainViewController.h"


// add a Project reference to cocos2d-ios.xcodeproj, using the dialog box and not the drag drop!! (xcode bug)
// (rstart xcode helps)
// modify your Build Settings. Set Always Search User Paths to YES, and add the Cocos2D source directory to the User Header Search Paths (as a recursive path)
// go to the Build Phases settings for your project, and add the Cocos2D library (libcocos2d.a) as a linked library


//see: http://www.notthepainter.com/full-cocos2d-uikit-integration/
//see: http://duckrowing.com/2011/11/09/using-the-singleton-pattern-in-objective-c-part-2/
//WORKS!: see: http://www.jerrodputman.com/2012/02/07/cocos2d-and-storyboards/#comment-707108201



@interface BrainDashMainViewController () {
    TheBrain            *theBrain;
    CCNode* myGame;
}
@end

@implementation BrainDashMainViewController


- (void)viewDidLoad
{
    CCDirector *director = [CCDirector sharedDirector];
    
    if([director isViewLoaded] == NO)
    {
        // Create the OpenGL view that Cocos2D will render to.
        CCGLView *glView = [CCGLView viewWithFrame:_joypadView.bounds //[[UIScreen mainScreen] bounds]
                                       pixelFormat:kEAGLColorFormatRGB565 //kEAGLColorFormatRGBA8
                                       depthFormat:0  //GL_DEPTH_COMPONENT24_OES
                                preserveBackbuffer:NO
                                        sharegroup:nil
                                     multiSampling:NO
                                   numberOfSamples:0];
        [glView setMultipleTouchEnabled:YES];
        
        // Assign the view to the director.
        director.view = glView;
        
        // Initialize other director settings.
        [director setAnimationInterval:1.0f/60.0f];
        //[director enableRetinaDisplay:YES];
    }
    
    // Set the view controller as the director's delegate, so we can respond to certain events.
    director.delegate = self;
    
    
    // Add the director as a child view controller of this view controller.
    [self addChildViewController:director];
    
    // Add the director's OpenGL view as a subview so we can see it.
    [_joypadView addSubview:director.view];
    [_joypadView sendSubviewToBack:director.view];
    
    
    // Finish up our view controller containment responsibilities.
    [director didMoveToParentViewController:self];
    
    HelloWorld* scene = [HelloWorld scene];
    // Run whatever scene we'd like to run here.
    if(director.runningScene)
        [director replaceScene:scene];
    else
        [director runWithScene:scene];
    
    
    
    theBrain = [TheBrain sharedInstance];
    
}




- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
    theBrain=nil;
}

#pragma mark - Flipside View

- (void)flipsideViewControllerDidFinish:(BrainDashFlipsideViewController *)controller
{
    [self dismissViewControllerAnimated:YES completion:nil];
}

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
    if ([[segue identifier] isEqualToString:@"showAlternate"]) {
        [[segue destinationViewController] setDelegate:self];
    }
}



- (void)viewDidUnload
{
    [super viewDidUnload];
    
    [[CCDirector sharedDirector] setDelegate:nil];
}


-(BOOL)shouldAutorotateToInterfaceOrientation: (UIInterfaceOrientation)interfaceOrientation

{
    
    return UIInterfaceOrientationIsLandscape(interfaceOrientation);
    
}

@end
