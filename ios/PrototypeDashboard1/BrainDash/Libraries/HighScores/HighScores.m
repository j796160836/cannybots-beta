//
//  HighScores.m
//  HighScores
//
//  Created by Faizan Aziz on 15/04/10.
//  Copyright 2010 __MyCompanyName__. All rights reserved.
//

#import "HighScores.h"


@implementation HighScores

const int MAX_HIGHSCORES = 10;

+ (void)addNewHighScore:(NSDictionary *)aScoreDictionary postGlobally:(BOOL)canPostGlobally withDelegate:(id)aDelegate{
	
	NSNumber *score = [aScoreDictionary objectForKey:@"score"];
	if( score == nil ){
		NSLog(@"Error: the key score must be present in the dictionary, can't add score");
		return;
	}
	
	NSMutableArray *localHighScores = [[NSMutableArray alloc] init];
	int totalScore = [score intValue];
	[HighScores updateLocalScoresWithArray:localHighScores completion:^{
		int tempTotalScore, count = [localHighScores count];
		BOOL didInsert = NO;
		
		for ( int i= 0 ; i < count; i++ ){
			tempTotalScore = [[[localHighScores objectAtIndex:i] objectForKey:@"score"] intValue];
			if(totalScore < tempTotalScore){
				[localHighScores insertObject:aScoreDictionary atIndex:i];
				didInsert = YES;
				break;
			}
		}
		
		if( !didInsert )
			[localHighScores addObject:aScoreDictionary];
		count++;
		
		while( count > MAX_HIGHSCORES ){
			[localHighScores removeLastObject];
			count--;
		}
		
		[localHighScores writeToFile:[HighScores highScoresFilePath] atomically:YES];
	}];
	
}

+ (void)clearLocalHighScores{
	NSArray *temp = [[NSArray alloc] init];
	[temp writeToFile:[HighScores highScoresFilePath] atomically:YES];
}

+ (NSString *)highScoresFilePath {
	NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
	return [[paths objectAtIndex:0] stringByAppendingPathComponent:@"HighScoresFile.plist"];	
}

+ (void)updateLocalScoresWithArray:(NSMutableArray*)aHighScoresArray completion:(void (^) ())aCompletionBlock{
	[aHighScoresArray removeAllObjects];
	
	NSArray *tHighScoresArray = [[NSArray alloc] initWithContentsOfFile:[HighScores highScoresFilePath]];
	[aHighScoresArray addObjectsFromArray:tHighScoresArray];
	
	aCompletionBlock();
}

@end
