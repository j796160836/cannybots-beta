//
//  HighScoresViewController.h
//  leaderboard
//
//  Created by Faizan Aziz on 22/04/10.
//  Copyright 2010 __MyCompanyName__. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "HighScores.h"

@interface HighScoresViewController : UIViewController <UITableViewDelegate, UITableViewDataSource, UITextFieldDelegate> {
	NSMutableArray *highScoresArray;
	
}

@property (nonatomic, retain) UITableView *highScoreTable;
@property (nonatomic, retain) UITextField *scoreField;
@property (nonatomic, retain) UITextField *nameField;

-(void)loadLocalScores;

- (IBAction)postScore;

@end
