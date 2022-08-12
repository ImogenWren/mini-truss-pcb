# autoDelay_library *(v2.0.0)*
#### Built and maintained by [Imogen Heard](mailto:pangalactictech@gmail.com "Email me!") on behalf of [Pan Galactic Tech](https://PanGalacticTech.com)

/*--------------------------------------autoDelay.h Library -------------------------------

   Simple method for timing events without using interrupt based delay functions.

    autoDelay != delay();

    functions include delays passed values in:
    Micro & Milliseconds,
    Seconds, & Minutes.
    
    Makes timing events a piece of cake.    
    
*/

# New To Version 2.0.0

* Added delayEvent to handle timed events with a set number of times to repeat.
______________________________________________



## Features
* Easy library for timing events using an object orientated methods.
* set up instance of autoDelay for each unique timer required.
* Manage program execution without managing hardware timer reads
* Methods included for Microseconds, Milliseconds, Seconds and Minutes of delay times.
* Bin the delay(); function that robs you of time better used for other purposes.
* Enable more complex programs by seperating time dependent events from each others execution.


__________________________________________________________________________________________________________

## Getting Started
### Deployment
1. Clone or Download library folder into Arduino IDE libraries folder.
2. Extract files from zipped folder if required.
3. Open Arduino IDE and select from example programs.

___________________________________________________________________________________________________________

## Use:


### Precompiler Requirements:

`#include <autoDelay.h> ` <br> <br>

___________________________________________________________________________________________________________

### Object Constructors

Set up instance for each different timer required by giving it a unique name:

`autoDelay timerOne;` <br>

`autoDelay timerTwo;` <br>

_________________________________________________________________________________________________________

### Setup Functions:

No Setup functions required.

<br>

___________________________________________________________________________________________________________

### Loop Functions:

### Infinatly Repeating Delays

Methods for infinatly repeating behaviours are used in the main loop as such:

In main loop call if statement with

`if (timerOne.millisDelay(long milliseconds)){` <br>
`// Call timed events here`<br>
`}                    `   <br> <br>

Repleace .millisDelay() with alternative methods for delays measured in other units.

>   Milliseconds =  .millisDelay(long milliseconds)  
>   Microseconds =  .microsDelay(long microseconds)  
>   Seconds      =  .secondsDelay(long seconds)  
>   Minutes      =  .minutesDelay(long minutes)  

___________________________________________________________________________________________________________

### Finite Repeating delayEvents

Delay events are designed to give you more control over program execution. We can now
start and stop the delayed behaviours using other control methods, and set behaviours to 
repeat a set number of times before stopping.

The implementation inside the main loop is the same as other autoDelay methods:

`if (timerOne.millisDelayEvent(long milliseconds)){` <br>
`// Call timed events here`<br>
`}                    `   <br> <br>

Repleace .millisDelayEvent() with alternative methods for delays measured in other units.

>   Milliseconds =  .millisDelayEvent(long milliseconds)  
>   Microseconds =  .microsDelayEven(long microseconds)  
>   Seconds      =  .secondsDelayEvent(long seconds)  
>   Minutes      =  .minutesDelayEvent(long minutes)  

<br>

### Starting Delay Events

With our main loop set up to respond to delayEvents, we can now add control methods to trigger 
these events.

The most useful of these is 

`timerOne.startDelay(repeats);         // This will repeat delayEvent (repeats) number of times.` <br>

This method will start the delayEvent, and repeat this event for the number of times passed as 
an argument. Calling this method will overwrite the previous state of the eventCounter. If this is not
intended, use .restartDelay() method.<br>

By passing any negative integer to this method, the delayEvent is triggered in infinate repeat mode,
and will continue until a stop method is called, or a finite number of repeat events are added to the
counter <br>

`timerOne.startDelay(-1);              // This will repeat delayEvent indefinatly`<br>

Indefinate reapeating behaviour can also be triggered by calling the method

`timerOne.startIndefinate();`<br>

This is functionally identical to the method above.<br><br>

Method to restart a stopped or paused delay while adding additional events to the counter.

`timerOne.restartDelay(additional);    // This starts or restarts delayEvent while adding additional events to the counter (default is 0)` <br>

<br>

### Stopping & Pausing Delay Events

delayEvents will stop when their eventCounter reaches zero. 

To stop delay events before their counter reaches zero, or to stop indefinatly repeating events, we can call the method

`timerOne.stopDelay(resetRepeats);` <br>

Default behaviour of this method resets eventCounter to zero. <br>

Passing false to this method will not reset the eventCounter, <br>
and is functionally the same as calling the following pauseDelay method. <br><br>

To pause delayEvents while keeping the eventCounter variable at its current state, we can use the method

`timerOne.pauseDelay();`<br><br>

### Adding & Subtracting to eventCounter

To change the number of events left in eventCounter, we can call the method

`timerOne.addCounter(additional);`<br>

The default behaviour is to add one to the number, however it can be passed any positive or negative integer in range,
enabling you to both add & remove events from the delayEvent eventCounter. This method is safe, if a negative integer is passed
which would take eventCounter into negative figures, it will reset it to zero. As such indefinate repeating behaviour cannot
inadvertantly be triggered through use of this method.

This method works while delayEvents are active, stopped or paused. <br><br><br>

<p>I hope you find this library useful. Good luck using it and please report any bugs or issues you find. </p>




___________________________________________________________________________________________________________

### Advanced Uses:




<br>
<br>

___________________________________________________________________________________________________________
___________________________________________________________________________________________________________

### Limitations:

* As interrupts are avoided entirely, timing events may not be executed in a priority order, leading to
less accurate triggering of timed events. 

* If more accurate timing is a priority please use [interrupt_autoDelay]( none ) (Coming Soon)


<br>

### Known Issues V2.0.0
  - Please report any bugs or issues found.

### Known Issues V1.0.0
  - Please report any bugs or issues found.

<br>

### Features to be Added 

<br>
<br>

## About
### Built With
* [Arduino IDE](https://www.arduino.cc/) - Default IDE


### Author
#### Imogen Heard
* [LinkedIn](https://www.linkedin.com/in/declan-heard-91103b58/](https://www.linkedin.com/in/imogen-heard-91103b58/)
* Email: [Personal](mailto:mogen.wren91@gmail.com "imogen.wren91@gmail.com") | [Company](mailto:PanGalacticTech@gmail.com "PanGalacticTech@gmail.com")
* [Github Profile](https://github.com/PanGalacticTech)




