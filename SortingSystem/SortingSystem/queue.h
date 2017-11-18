/* queue.h */

/* Type definitions */
typedef struct item{
	char metal;
	char reflective;
	char type;
    char stage;
	struct item* next;
} item;

typedef struct queue{
	item* head;			// The ptr to the head of the queue
	item* tail;			// The ptr to the tail of the queue
}queue;

/* Subroutine headers */
item*	initItem	();
queue*	initQueue	();
void 	clearQueue	(queue* q);
void	deleteItem	(item* i);
void	resetItem	(item* i);
void	updateItem	(item* i, char metal, char reflective, char type, char stage);
char	itemType	(item* i);
void 	enqueue		(queue* q, item* newItem);
item* 	dequeue		(queue* q);
item	firstValue	(queue* q);
item	lastValue	(queue* q);
int 	isEmpty		(queue* q);
int 	size		(queue* q);


