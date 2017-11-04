/* Solution Set for the LinkedQueue.c */
/* 	
	Course		: UVic Mechatronics 458
	Milestone	: 3
	Title		: Data structures for MCUs and the Linked Queue Library

	Name 1:					Student ID:
	Name 2:					Student ID:
*/

/* include libraries */
#include <stdlib.h>
#include <avr/io.h>
#include "LinkedQueue.h" 

/* main routine */
/*
int main(){	

	link *head;			// The ptr to the head of the queue 
	link *tail;			// The ptr to the tail of the queue 
	link *newLink;		// A ptr to a link aggregate data type (struct) 
	link *rtnLink;		// same as the above 
	element eTest;		// A variable to hold the aggregate data type known as element 

	DDRC = 0xFF; 		// Used for debugging purposes only 
				

	rtnLink = NULL;
	newLink = NULL;

	setup(&head, &tail);

	// Initialize a new link here 
	initLink(&newLink);
	newLink->e.itemCode = 3;
	newLink->e.stage = 4;
	enqueue(&head, &tail, &newLink);
	PORTC = head->e.itemCode;
	PORTC = tail->e.stage;

	initLink(&newLink);
	newLink->e.itemCode = 5;
	newLink->e.stage = 6;
	enqueue(&head, &tail, &newLink);
	PORTC = head->e.itemCode;
	PORTC = tail->e.stage;

	initLink(&newLink);
	newLink->e.itemCode = 7;
	newLink->e.stage = 8;
	enqueue(&head, &tail, &newLink);
	PORTC = head->e.itemCode;
	PORTC = tail->e.stage;

	PORTC = 0x00;

	// Tests to see if firstValue works 
	eTest = firstValue(&head);
	PORTC = eTest.itemCode;
	PORTC = 0x00;

	// Tests if Size works 
	PORTC = size(&head, &tail);

	// Tests if dequeue works - Uncomment to use
	dequeue(&head, &rtnLink); // remove the item at the head of the list 
	PORTC = rtnLink->e.itemCode;
	dequeue(&head, &rtnLink); // remove the item at the head of the list 
	PORTC = rtnLink->e.itemCode;
	dequeue(&head, &rtnLink); // remove the item at the head of the list 
	PORTC = rtnLink->e.itemCode;
	
	// Tests is empty 
	PORTC = isEmpty(&head);

	// Tests to see if clearQueue works
	clearQueue(&head, &tail);
	PORTC = size(&head, &tail);

	PORTC = isEmpty(&head);


	return(0);
}//main
*/

/***************************** SUBROUTINES ********************************************/

/**************************************************************************************
* DESC: initializes the linked queue to 'NULL' status
* INPUT: the head and tail pointers by reference
*/
void setup(link **h,link **t){
	*h = NULL;		/* Point the head to NOTHING (NULL) */
	*t = NULL;		/* Point the tail to NOTHING (NULL) */
	return;
}/*setup*/


/**************************************************************************************
* DESC: This initializes a link and returns the pointer to the new link or NULL if error 
* INPUT: the head and tail pointers by reference
*/
void initLink(link **newLink){
	//link *l;
	*newLink = malloc(sizeof(link));
	(*newLink)->next = NULL;
	return;
}/*initLink*/

void initQueue(queue* newQueue){
	
}


/****************************************************************************************
*  DESC: Accepts as input a new link by reference, and assigns the head and tail		
*  of the queue accordingly				
*  INPUT: the head and tail pointers, and a pointer to the new link that was created 
*/
/* will put an item at the tail of the queue */
void enqueue(link **h, link **t, link **nL){

	if (*t != NULL){
		/* Not an empty queue */
		(*t)->next = *nL;
		*t = *nL; //(*t)->next;
	}/*if*/
	else{
		/* It's an empty Queue */
		//(*h)->next = *nL;
		//should be this
		*h = *nL;
		*t = *nL;
	}/* else */
	return;
}/*enqueue*/


/**************************************************************************************
* DESC : Removes the link from the head of the list and assigns it to deQueuedLink
* INPUT: The head and tail pointers, and a ptr 'deQueuedLink' 
* 		 which the removed link will be assigned to
*/
/* This will remove the link and element within the link from the head of the queue */
void dequeue(link **h, link **t, link **deQueuedLink){
	*deQueuedLink = *h;	// Will set to NULL if Head points to NULL
	/* Ensure it is not an empty queue */
	if (*h != NULL){
		*h = (*h)->next;
	}/*if*/
	if(*h == NULL) *t = NULL; // if that was the last element in the queue, set tail to NULL as well
	return;
}/*dequeue*/


/**************************************************************************************
* DESC: Peeks at the first element in the list
* INPUT: The head pointer
* RETURNS: The element contained within the queue
*/
/* This simply allows you to peek at the head element of the queue and returns a NULL pointer if empty */
element firstValue(link **h){
	return((*h)->e);
}/*firstValue*/


/**************************************************************************************
* DESC: deallocates (frees) all the memory consumed by the Queue
* INPUT: the pointers to the head and the tail
*/
/* This clears the queue */
void clearQueue(link **h, link **t){

	link *temp;

	while (*h != NULL){
		temp = *h;
		*h=(*h)->next;
		free(temp);
	}/*while*/
	
	/* Last but not least set the tail to NULL */
	*t = NULL;		

	return;
}/*clearQueue*/


/**************************************************************************************
* DESC: Checks to see whether the queue is empty or not
* INPUT: The head pointer
* RETURNS: 1:if the queue is empty, and 0:if the queue is NOT empty
*/
/* Check to see if the queue is empty */
char isEmpty(link **h){
	/* ENTER YOUR CODE HERE */
	return(*h == NULL);
}/*isEmpty*/


/**************************************************************************************
* DESC: Obtains the number of links in the queue
* INPUT: The head and tail pointer
* RETURNS: An integer with the number of links in the queue
*/
/* returns the size of the queue*/
int size(link **h, link **t){

	link 	*temp;			/* will store the link while traversing the queue */
	int 	numElements;

	numElements = 0;

	temp = *h;			/* point to the first item in the list */

	while(temp != NULL){
		numElements++;
		temp = temp->next;
	}/*while*/
	
	return(numElements);
}/*size*/

