#include <stdlib.h>
#include <avr/io.h>
#include "queue.h" 


/**************************************************************************************
* DESC: This initializes a link and returns the pointer to the new link or NULL if error 
* INPUT: the head and tail pointers by reference
*/
link* initLink(){
	link* newLink = malloc(sizeof(link));
	newLink->next = NULL;
	newLink->e.metal = 0;
	newLink->e.reflective = 0;
	newLink->e.type = 0;
	newLink->e.stage = 0;
	return newLink;
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
void enqueue(queue* q, link* newLink){
	if (q->tail != NULL){
		/* Not an empty queue */
		q->tail->next = newLink;
		q->tail = newLink;
	}/*if*/
	else{
		/* It's an empty Queue */
		q->head = newLink;
		q->tail = newLink;
	}/* else */
	return;
}/*enqueue*/


/**************************************************************************************
* DESC : Removes the link from the head of the list and assigns it to deQueuedLink
* RETURNS: Pointer to the dequeued link
*/
link* dequeue(queue* q){
	link* deQueuedLink = q->head;	// Will set to NULL if Head points to NULL
	/* Ensure it is not an empty queue */
	if (q->head != NULL){
		q->head = q->head->next;
	}/*if*/
	if(q->head == NULL) q->tail = NULL; // if that was the last element in the queue, set tail to NULL as well
	return deQueuedLink;
}/*dequeue*/


/**************************************************************************************
* DESC: Peeks at the first or last element in the list
* RETURNS: The element contained within the queue
*/
element firstValue(queue* q){
	return(q->head->e);
}//firstValue

element lastValue(queue* q){
	return (q->tail->e);
}//lastValue


/**************************************************************************************
* DESC: deallocates (frees) all the memory consumed by the Queue
* INPUT: the pointers to the head and the tail
*/
void clearQueue(queue* q){
	link *temp;
	
	while (q->head != NULL){
		temp = q->head;
		q->head = q->head->next;
		free(temp);
	}/*while*/

	q->tail = NULL;		
	return;
}//clearQueue

void deleteLink(link* L){
	free(L);
}

void resetLink(link* L){
	L->e.metal = 0;
	L->e.reflective = 0;
	L->e.type = 0;
	L->e.stage = 0;
}

void updateLink(link* L, int metal, int reflective, int type, int stage){
	L->e.metal = metal;
	L->e.reflective = reflective;
	L->e.type = type;
	L->e.stage = stage;
}

int elementType(link* L){
	L->e.type = L->e.metal + L->e.reflective;
	return (L->e.type);
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
	link* temp = q->head;			
	int numElements = 0;

	while(temp != NULL){
		numElements++;
		temp = temp->next;
	}/*while*/
	
	return(numElements);
}//size

