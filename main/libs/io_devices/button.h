#ifndef BUTTON_H
#define BUTTON_H
class Button {
    private:
    int pin = 15;

    int button_state = LOW;
    int previous_state = LOW;

    unsigned int pressed_time = 0;
    unsigned int await_time = 0;

    bool one_pressed = false;
    bool double_pressed = false;
    bool hold_pressed = false;
    bool long_hold_pressed = false;

    public:
    Button() {}
    
    Button(int pin) {
        this->pin = pin;
    }

    Button(int pin, int red_light_pin, int green_light_pin, int blue_light_pin) {
        this->pin = pin;
    }
    
    void activate() {
        // initialize the pushbutton pin as an input:
        pinMode(pin, INPUT);
    }

    bool pressed() {
        button_state = digitalRead(pin);
        if (button_state == HIGH) {

            if (previous_state == LOW) {
                pressed_time = millis();
            } else if (millis() - pressed_time > 3000) {
                return true;
            }
        }
        previous_state = button_state;
        return false;
    }

    void run() {
        button_state = digitalRead(pin);

        if (button_state == HIGH) {

            if (previous_state == LOW) {

                if (one_pressed) {
                    double_pressed = millis() - await_time >= 300;
                } else {
                    one_pressed = true;
                }

                pressed_time = millis();
            }

        
            long_hold_pressed = millis() - pressed_time >= 10000;
        } else if (previous_state == HIGH) {
            await_time = millis();
            hold_pressed = millis() - pressed_time >= 3000;
        }

        previous_state = button_state;
    }

    bool is_one_pressed() {return one_pressed;}
    bool is_double_pressed() {return double_pressed;}
    bool is_hold_pressed() {return hold_pressed;}
    bool is_long_hold_pressed() {return long_hold_pressed;}

    bool one_reset() {return one_pressed = false;}
    bool double_reset() {return double_pressed = false;}
    bool hold_reset() {return hold_pressed = false;}
    bool long_hold_reset() {return long_hold_pressed = false;}
};

#endif