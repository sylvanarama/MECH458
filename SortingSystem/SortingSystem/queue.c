#include <stdlib.h>
#include <avr/io.h>
#include "queue.h" 


/**************************************************************************************
* DESC: This initializes a link and returns the pointer to the new link or NULL if error 
* INPUT: the head and tail pointers by reference
*/
item* initItem(){
	item* newItem = malloc(sizeof(item));
	newItem->next = NULL;
	newItem->metal = 0;
	newItem->reflective = 0;
	newItem->type = 7;		// initial value = 7 for testing
	newItem->stage = 0;
	return newItem;
}//initLink

queue* initQueue(){
	queue* q = malloc(sizeof(queue));
	q->head = NULL;	
	q->tail = NULL;	
	return q;
}//initQueue

/****************************************************************************************
*  DESC: Accepts as input a new link by reference, and assigns the head and tail		
*  of the queue accordingly (Will put an item at the tail of the queue)		
*/
void enqueue(queue* q, item* newItem){
	if (q->tail != NULL){
		/* Not an empty queue */
		q->tail->next = newItem;
		q->tail = newItem;
	}/*if*/
	else{
		/* It's an empty Queue */
		q->head = newItem;
		q->tail = newItem;
	}/* else */
	return;
}/*enqueue*/


/**************************************************************************************
* DESC : Removes the link from the head of the list and assigns it to deQueuedLink
* RETURNS: Pointer to the dequeued link
*/
item* dequeue(queue* q){
	item* deQueuedItem = q->head;	// Will set to NULL if Head points to NULL
	/* Ensure it is not an empty queue */
	if (q->head != NULL){
		q->head = q->head->next;
	}/*if*/
	if(q->head == NULL) q->tail = NULL; // if that was the last element in the queue, set tail to NULL as well
	return deQueuedItem;
}/*dequeue*/


/**************************************************************************************
* DESC: Peeks at the first or last item in the list
* RETURNS: The element contained within the queue
*/
item firstValue(queue* q){
	return *(q->head);
}//firstValue

item lastValue(queue* q){
	return *(q->tail);
}//lastValue


/**************************************************************************************
* DESC: deallocates (frees) all the memory consumed by the Queue
* INPUT: the pointers to the head and the tail
*/
void clearQueue(queue* q){
	item *temp;
	
	while (q->head != NULL){
		temp = q->head;
		q->head = q->head->next;
		free(temp);
	}/*while*/

	q->tail = NULL;		
	return;
}//clearQueue

void deleteItem(item* i){
	free(i);
}

void resetItem(item* i){
	i->metal = 0;
	i->reflective = 0;
	i->type = 0;
	i->stage = 0;
}

void updateItem(item* i, char metal, char reflective, char type, char stage){
	i->metal = metal;
	i->reflective = reflective;
	i->type = type;
	i->stage = stage;
}

char itemType(item* i){
	i->type = i->metal + i->reflective;
	return (i->type);
}

/**************************************************************************************
* DESC: Checks to see whether the queue is empty or not
* RETURNS: 1:if the queue is empty, and 0:if the queue is NOT empty
*/
/* Check to see if the queue is empty */
int isEmpty(queue* q){
	return(q->head == NULL);
}//isEmpty


/**************************************************************************************
* DESC: Obtains the number of links in the queue
* RETURNS: An integer with the number of links in the queue
*/
int size(queue* q){
	item* temp = q->head;			
	int numItems = 0;

	while(temp != NULL){
		numItems++;
		temp = temp->next;
	}/*while*/
	
	return(numItems);
}//size

