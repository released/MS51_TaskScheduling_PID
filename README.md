# MS51_TaskScheduling_PID
 MS51_TaskScheduling_PID

update @ 2020/01/10

1. Add PID test , with PID_INCREMENTAL define

2. Use ADC to generate seed , and use rand() to create random number , for test PID procedure

3. check log message , value_approach after PID count , will close to value_current/target each loop , until meet target value

ex : 

,current: 759, approach : 770

,current: 759, approach : 769

,current: 759, approach : 768

,current: 759, approach : 767

,current: 759, approach : 766

,current: 759, approach : 765

,current: 759, approach : 764

----------------------------------------

platform : update MS51 driver to Keil_V1.00.003 , test in MS51 TSSOP20 EVM

1. using task (code base from internet) and array list , to control function and simply the procedure

2. current task max number , refer to define MAXTASKS

3. to add new function into task list

- add new function , task template refer to current task

- insert this new task into TaskList array , will automatically start the new task

4. to porting to other platform ,

- insert UpdateTimers() into TIMER interrupt IRQ with 1 ms

- calling TaskSchedulerInit() and TaskSchedulerStart() instead of while (1) after function init finish