/* 
    Simple state machine used for keeping track of tasks performed
    by the green pick station.

    Ted Lutkus
    6/26/20
*/

#define STARTUP 0
#define CALIBRATE_NEW_ITEM 1
#define PACKING_ITEM 2
#define READY_FOR_PICKPOINT 3


class StateMachine {
private:
    int current_state;
public:
    // Initialize state machine
    StateMachine() {
        this->current_state = STARTUP;
    }

    // Set the state machine's state
    void set_state(int new_state) {
        this->current_state = new_state;
    }

    // Get the state machine's state
    int get_state() {
        return this->current_state;
    }
};