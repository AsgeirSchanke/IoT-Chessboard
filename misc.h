/*
 * misc.h
 *
 * Created: 12.12.2016 14:39:26
 *  Author: M44001
 */


#ifndef MISC_H_
#define MISC_H_


/*
 * Macro used to guarantee read-modify-write operations
 * are carried out atomically to avoid concurrency
 * issues with ISRs
 */
#define ATOMIC( operation ) cli(); operation; sei()


#endif /* MISC_H_ */