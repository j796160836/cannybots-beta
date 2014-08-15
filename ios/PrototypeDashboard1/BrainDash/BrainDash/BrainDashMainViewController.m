//
//  BrainDashMainViewController.m
//  BrainDash
//
//  Created by Wayne Keenan on 16/04/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import "BrainDashMainViewController.h"

//#import <CannybotsController.h>
//#import "CannybotsRacerGlu.h"
#import "BrainSpeakBLE.h"

// add a Project reference to cocos2d-ios.xcodeproj, using the dialog box and not the drag drop!! (xcode bug)
// (rstart xcode helps)
// modify your Build Settings. Set Always Search User Paths to YES, and add the Cocos2D source directory to the User Header Search Paths (as a recursive path)
// go to the Build Phases settings for your project, and add the Cocos2D library (libcocos2d.a) as a linked library


//see: http://www.notthepainter.com/full-cocos2d-uikit-integration/
//see: http://duckrowing.com/2011/11/09/using-the-singleton-pattern-in-objective-c-part-2/
//WORKS!: see: http://www.jerrodputman.com/2012/02/07/cocos2d-and-storyboards/#comment-707108201



@interface BrainDashMainViewController () {
    //CannybotsController* cb;
    BrainSpeakBLE* bsle;
    CCNode* myGame;
    NSTimer* lapTimer;
    double lapTimerStartTime;
    bool joystickMode;
    int leftMotorSpeed, rightMotorSpeed;
}
@end

@implementation BrainDashMainViewController


- (void)viewDidLoad
{
    CGAffineTransform sliderRotation = CGAffineTransformIdentity;
    sliderRotation = CGAffineTransformRotate(sliderRotation, -(M_PI / 2));
    _leftSpeedSlider.transform = sliderRotation;
    _rightSpeedSlider.transform = sliderRotation;
    
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
    
    
    //cb = [CannybotsController sharedInstance];
    bsle     = [BrainSpeakBLE sharedInstance];
    joystickMode = true;
    
    [self startLapTimer];
    
}

- (void) reloadScene {
    
    CCScene *currentScene = [CCDirector sharedDirector].runningScene;
    CCScene *newScene = [[[currentScene class] alloc] init];
    [[CCDirector sharedDirector] replaceScene:newScene];
    
}



- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
    //cb=nil;
}

#pragma mark - Flipside View

/*- (void)flipsideViewControllerDidFinish:(BrainDashFlipsideViewController *)controller
{
    [self dismissViewControllerAnimated:YES completion:nil];
}
 */

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


-(BOOL)shouldAutorotateToInterfaceOrientation: (UIInterfaceOrientation)interfaceOrientation {
    return UIInterfaceOrientationIsLandscape(interfaceOrientation);
}


// Gesture recognisers

- (IBAction)longPressDetected:(UIGestureRecognizer *)sender {
    _statusLabel.text = @"Long Press";
}


- (IBAction)swipeDetected:(UIGestureRecognizer *)sender {
    _statusLabel.text = @"Right Swipe";
}

- (IBAction)tapDetected:(UIGestureRecognizer *)sender {
    _statusLabel.text = @"Double Tap";
}

- (IBAction)pinchDetected:(UIGestureRecognizer *)sender {
    static double lastTime = 0;
    
    double CurrentTime = CFAbsoluteTimeGetCurrent();//CACurrentMediaTime();
    if ( (CurrentTime - lastTime) < ((float)1/5)) {
        return;
    } else {
        lastTime = CurrentTime;
    }
    
    
    CGFloat scale =    [(UIPinchGestureRecognizer *)sender scale];
    CGFloat velocity = [(UIPinchGestureRecognizer *)sender velocity];
    
    NSString *resultString = [[NSString alloc] initWithFormat:
                              @"Pinch - scale = %f, velocity = %f",
                              scale, velocity];
    _statusLabel.text = resultString;
}

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
- (IBAction)rotationDetected:(UIGestureRecognizer *)sender {
    static double lastTime = 0;
    
    double CurrentTime = CFAbsoluteTimeGetCurrent();//CACurrentMediaTime();
    if ( (CurrentTime - lastTime) < ((float)1/5)) {
        return;
    } else {
        lastTime = CurrentTime;
    }
    
    CGFloat radians =  [(UIRotationGestureRecognizer *)sender rotation];
    CGFloat velocity = [(UIRotationGestureRecognizer *)sender velocity];
    
    NSString *resultString = [[NSString alloc] initWithFormat:
                              @"Rotation - Radians = %f, velocity = %f",
                              radians, velocity];
    _statusLabel.text = resultString;
    
    
    
    //uint16_t vel =200;
    //uint16_t pos =200 + RADIANS_TO_DEGREES(radians)*2;
    
}

- (IBAction)panDetected:(UIGestureRecognizer *)sender {
    
    CGPoint trans    = [(UIPanGestureRecognizer*)sender translationInView:_gestureView];
    CGPoint velocity = [(UIPanGestureRecognizer*)sender velocityInView:_gestureView];

    NSString *resultString = [[NSString alloc] initWithFormat:
                              @"Pan - trans = %@, velocity = %@",
                              NSStringFromCGPoint(trans), NSStringFromCGPoint(velocity)];
    _statusLabel.text = resultString;
}


- (IBAction)leftSpeedChanged:(UISlider*)sender {
    leftMotorSpeed=(int)(sender.value);
    //[cb callMethod:&RACER_JOYAXIS p1:leftMotorSpeed p2:rightMotorSpeed];
    
}
- (IBAction)rightSpeedChanged:(UISlider*)sender {
    rightMotorSpeed=(int)(sender.value);
    //[cb callMethod:&RACER_JOYAXIS p1:leftMotorSpeed p2:rightMotorSpeed];
}


- (IBAction)modeChanged:(UISegmentedControl *)sender {
    
    if ( 0 == sender.selectedSegmentIndex) {
        joystickMode=true;
        _joypadView.hidden=false;
        _leftSpeedSlider.hidden=true;
        _rightSpeedSlider.hidden=true;
        //[cb callMethod:&RACER_LINEFOLLOWING_MODE p1:1];
        //[cb callMethod:&RACER_TANKCONTROL_MODE p1:0 ];

    } else if ( 1 == sender.selectedSegmentIndex) {
        //[cb callMethod:&RACER_LINEFOLLOWING_MODE p1:0 ];
    } else if ( 2 == sender.selectedSegmentIndex) {
        joystickMode=false;
        _joypadView.hidden=true;
        _leftSpeedSlider.hidden=false;
        _rightSpeedSlider.hidden=false;
        
        //[cb callMethod:&RACER_TANKCONTROL_MODE p1:1 ];
    }
    
}

- (IBAction)restLapStatsPressed:(id)sender {
}


- (void) lapCompleted:(float)time {
    NSLog(@"Lap time %f", time);
    
   // int laps = _lapCounterTextField.text.integerValue;
    //_lapCounterTextField.text = [NSString stringWithFormat:@"%d", laps++];
    
    _lapTimeTextField.text = [NSString stringWithFormat:@"%0.2f", time];
    _bestLapTimeTextField.text = [NSString stringWithFormat:@"%0.2f", time];

    [lapTimer invalidate];
}


- (void) lapCount:(int)count {
    NSLog(@"Lap count %d", count);
    //int laps = _lapCounterTextField.text.integerValue;
    _lapCounterTextField.text = [NSString stringWithFormat:@"%d", count];
}

- (void) startLapTimer {
    NSLog(@"Lap time %f", time);
    lapTimerStartTime = CACurrentMediaTime();
    [lapTimer invalidate];
    lapTimer=nil;
    lapTimer = [NSTimer scheduledTimerWithTimeInterval:0.01 target:self selector:@selector(updateTimer) userInfo:nil repeats:YES];
}

- (void) lapStop:(int)lapCount {
    NSLog(@"Lap stop %d", lapCount);
    //_bestLapTimeTextField.text = [NSString stringWithFormat:@"%d", lapCount];
    [lapTimer invalidate];
    lapTimer=nil;
}


- (void) updateTimer {
    double currentLapTime = CACurrentMediaTime() - lapTimerStartTime;
    _lapTimeTextField.text = [NSString stringWithFormat:@"%0.2f", currentLapTime];

}


- (void) viewDidAppear:(BOOL)animated {
    NSLog(@"viewDidAppear");
    /*
    [cb registerHandler:&RACER_LINEFOLLOWING_MODE withBlockFor_INT16_3: ^(int16_t p1, int16_t p2, int16_t p3)
    {
        if (joystickMode) {
            [self.modeSegment setSelectedSegmentIndex:p1>0?1:0];
        } else {
            [self.modeSegment setSelectedSegmentIndex:p1>0?1:2];

        }
    }];

    [cb registerHandler:&LAPCOUNTER_GETREADY withBlockFor_INT16_1: ^(int16_t p1)
     {
         [self startLapTimer ];
     }];
    
    [cb registerHandler:&LAPCOUNTER_LAPTIME withBlockFor_INT16_1: ^(int16_t p1)
     {
         [self lapCompleted: p1/1000 ];
     }];

    [cb registerHandler:&LAPCOUNTER_LAPCOUNT withBlockFor_INT16_1: ^(int16_t p1)
     {
         [self lapCount:p1];
     }];
    
    [cb registerHandler:&LAPCOUNTER_STOP withBlockFor_INT16_1: ^(int16_t p1)
     {
         [self lapStop:p1];
     }];
    */
    [self reloadScene];
}


- (void) viewWillDisappear:(BOOL)animated {
    //[cb deregisterHandler:&RACER_LINEFOLLOWING_MODE];
}


@end
