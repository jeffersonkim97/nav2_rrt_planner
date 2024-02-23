#ifndef CONSTANTS_H
#define CONSTANTS_H

const float WORLD_WIDTH = 100.0;
const float WORLD_LENGTH = 100.0;
const float WORLD_HEIGHT = 100.0;
const float BOT_RADIUS = 1.0;
const float START_POS_X = 1.0;
const float START_ORIENT = 0.785;
const float START_POS_Y = 1.0;
const float START_POS_Z = 1.0;
const float END_POS_X = 90.0;
const float END_POS_Y = 90.0;
const float END_POS_Z = 90.0;
const float NODE_RADIUS = 1.0;
const float END_DIST_THRESHOLD = 1;
const float BOT_CLEARANCE = 1.5 * BOT_RADIUS;
const float BOT_TURN_RADIUS = 7;
const float RRTSTAR_NEIGHBOR_FACTOR = 0;
const bool BOT_FOLLOW_DUBIN = true;

#endif // CONSTANTS_H