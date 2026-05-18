#ifndef FSM_H
#define FSM_H

#include <stdbool.h>

typedef struct Fsm_State Fsm_State_t;

struct Fsm_State
{
    const char *name;
    void (*enter)(void);
    void (*exit)(void);
    Fsm_State_t *(*update)(void);
};

typedef struct
{
    const char *name;
    Fsm_State_t *state;
} Fsm_t;

bool Fsm_Initialize(Fsm_t *const fsm, Fsm_State_t *const state, const char *const name);
void Fsm_Update(Fsm_t *const fsm);

#endif /* FSM_H */