//
//  HighScoresViewController.m
//  leaderboard
//
//  Created by Faizan Aziz on 22/04/10.
//  Copyright 2010 __MyCompanyName__. All rights reserved.
//

#import "HighScoresViewController.h"
#import "CannybotsController.h"
#import "CannyLapCounter.h"


@implementation HighScoresViewController

@synthesize highScoreTable, nameField, scoreField;

// Implement viewDidLoad to do additional setup after loading the view, typically from a nib.
- (void)viewDidLoad {
    [super viewDidLoad];
	
	highScoresArray = [[NSMutableArray alloc] init];
	
	//[self.view addSubview:highScoreTable];
	
	
	[nameField setDelegate:self];
	
	[self loadLocalScores];
}


- (BOOL)textFieldShouldReturn:(UITextField *)textField{
	[textField resignFirstResponder];
	return YES;
}



-(void)loadLocalScores{
	[highScoreTable setHidden:YES];
	
	[HighScores updateLocalScoresWithArray:highScoresArray completion:^{
		dispatch_async(dispatch_get_main_queue(), ^{
			[highScoreTable reloadData];
			[highScoreTable setHidden:NO];
		});
	}];
}

/*
-(IBAction)postScore{
	[nameField resignFirstResponder];
	[scoreField resignFirstResponder];
	
	if( [[nameField text] isEqualToString:@""] || [[scoreField text] isEqualToString:@""] ){
		UIAlertView *msg = 	[[UIAlertView alloc] initWithTitle:@"Check data" message:@"Make sure name and score is filled" delegate:nil cancelButtonTitle:@"Dismiss" otherButtonTitles:nil];
		[msg show];
	}
	
	NSMutableArray *temp = [[NSMutableArray alloc] init];
	[temp addObject:[nameField text]];
	[temp addObject:[NSNumber numberWithInt:[[scoreField text] intValue] ] ];
	
	NSMutableArray *temp1 = [[NSMutableArray alloc] init];
	[temp1 addObject:@"name"];
	[temp1 addObject:@"score"];
	
	[HighScores addNewHighScore:[NSDictionary dictionaryWithObjects:temp forKeys:temp1] postGlobally:NO withDelegate:self];

    [self loadLocalScores];
}
*/


//local high scores

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section {
	int x = tableView.frame.size.height/44;
	x++;
	if(x>[highScoresArray count])
		return x;
	
	return [highScoresArray count];
}


- (UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath {
	
	static NSString *HighScoreCellIdentifier = @"HighScoreCellIdentifier";
	UITableViewCell *cell = [tableView dequeueReusableCellWithIdentifier:HighScoreCellIdentifier];
	
	if (cell == nil) {
		cell = [[UITableViewCell alloc] initWithStyle:UITableViewCellStyleValue1 reuseIdentifier:HighScoreCellIdentifier];
	}
	
	int row = indexPath.row;
	NSDictionary *highScoreDict = nil;
	if( row < [highScoresArray count] )
		highScoreDict = [highScoresArray objectAtIndex:row];
	if( row%2 != 0)
		cell.backgroundColor = [UIColor colorWithRed:.9f green:.9f blue:.9f alpha:1];
	else
		cell.backgroundColor = [UIColor whiteColor];
	
	if(highScoreDict == nil ){
		cell.textLabel.text = @"";
		cell.detailTextLabel.text = @"";
	}
	else {
		
		cell.detailTextLabel.text = [NSString stringWithFormat:@"%0.2f", [[highScoreDict objectForKey:@"score"] intValue]/1000.0];
		
		NSString *temp = [[NSString alloc] initWithFormat:@"%d. %@", row + 1, [highScoreDict objectForKey:@"name"]];
		cell.textLabel.text = temp;
	}
	return cell;
}

- (void)didReceiveMemoryWarning {
	// Releases the view if it doesn't have a superview.
    [super didReceiveMemoryWarning];
	
	// Release any cached data, images, etc that aren't in use.
}

- (void)viewDidUnload {
}

- (void)dealloc {
	//[highScoreTable removeFromSuperview];
}

//////////////

- (void) viewDidAppear:(BOOL)animated {
    CannybotsController* cb = [CannybotsController sharedInstance];
    
    [cb registerHandler:LAPCOUNTER_LAPTIME withBlockFor_INT16_1: ^(int16_t p1)
     {
         [self recordLapTime: p1 ];
     }];
    
}


- (void) viewWillDisappear:(BOOL)animated {
    CannybotsController* cb = [CannybotsController sharedInstance];
    [cb deregisterHandler:LAPCOUNTER_LAPTIME];
}



- (IBAction)readyPressed:(id)sender {
  	[nameField resignFirstResponder];
    [self resetLapCounter];
}

- (IBAction)clearPressed:(id)sender {
    [HighScores clearLocalHighScores];
    [self loadLocalScores];
}


- (void) resetLapCounter {
    CannybotsController* cb = [CannybotsController sharedInstance];
    [cb callMethod:LAPCOUNTER_GETREADY p1:0];
    NSLog(@"Reset laptime");

}

- (void) recordLapTime:(float)time {
    NSLog(@"Lap time %f", time);
    
    NSMutableArray *temp = [[NSMutableArray alloc] init];
	[temp addObject:[nameField text]];
	[temp addObject:[NSNumber numberWithFloat:time]];
	
	NSMutableArray *temp1 = [[NSMutableArray alloc] init];
	[temp1 addObject:@"name"];
	[temp1 addObject:@"score"];
    
	[HighScores addNewHighScore:[NSDictionary dictionaryWithObjects:temp forKeys:temp1] postGlobally:NO withDelegate:self];
    
    [self loadLocalScores];
}



@end
