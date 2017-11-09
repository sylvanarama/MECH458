/* LinkedQueue.h */

/* Type definitions */
typedef struct {
	int metal;
	int reflective;
	int type;
	int stage; 	
} element;

typedef struct link{
	element		e;
	struct link* next;
} link;

typedef struct queue{
	link *head;			// The ptr to the head of the queue
	link *tail;			// The ptr to the tail of the queue
}queue;

/* Subroutine headers */
link*	initLink	();
queue*	initQueue	();
void 	clearQueue	(queue* q);
void	deleteLink	(link* L);
void	resetLink	(link* L);
void	updateLink	(link* L, int metal, int reflective, int type, int stage);
int		elementType	(link* L);
void 	enqueue		(queue* q, link* newLink);
link* 	dequeue		(queue* q);
element firstValue	(queue* q);
element lastValue	(queue* q);
int 	isEmpty		(queue* q);
int 	size		(queue* q);

