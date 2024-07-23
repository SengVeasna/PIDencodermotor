/*
 * DIFF.c
 *
 *  Created on: Oct 3, 2023
 *      Author: User
 */
#include "DIFF.h"
void Differentiator_Init(Differentiator *diff) {
/* Clear controller variables */
diff->prevInput = 0.0f;
diff->prevOutput = 0.0f;
diff->output = 0.0f;
}
float Differentiator_Update(Differentiator *diff, float input) {
 diff->output = (2.0f * (input - diff->prevInput)
+ (2.0f * diff->tau - diff->T) * diff->prevOutput)
 / (2.0f * diff->tau + diff->T);
/* Store error and measurement for later use */
/* Return controller output */
 diff->prevOutput = diff->output;
 diff->prevInput = input;
 return diff->output;
}

