//
//  HighScores.h
//  HighScores
//
//  Created by Faizan Aziz on 15/04/10.
//  Copyright 2010 __MyCompanyName__. All rights reserved.
//

//	To use this the dictionary must have a key called score

#import <Foundation/Foundation.h>

@interface HighScores : NSObject {
}

+ (void)addNewHighScore:(NSDictionary *)aScoreDictionary postGlobally:(BOOL)canPostGlobally withDelegate:(id)aDelegate;
+ (void)clearLocalHighScores;

+ (NSString *)highScoresFilePath;
+ (void)updateLocalScoresWithArray:(NSMutableArray*)aHighScoresArray completion:(void (^) ())aCompletionBlock;	//Used to get the latest glocal scores

@end
