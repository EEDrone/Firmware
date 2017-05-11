#ifndef CONFIG_H_
#define CONFIG_H_
// Task names
#define TEST_TASK_NAME        "TEST"
#define SYSTEM_TASK_NAME        "SYSTEM"
#define INIT_TASK_NAME        "INIT"
#define MAIN_TASK_NAME        "MAIN"
// Task priorities. Higher number higher priority
#define TEST_TASK_PRI        2
#define SYSTEM_TASK_PRI        2
#define INIT_TASK_PRI        5
#define MAIN_TASK_PRI        6

//Task stack sizes
#define TEST_TASK_STACKSIZE          configMINIMAL_STACK_SIZE
#define SYSTEM_TASK_STACKSIZE         (2* configMINIMAL_STACK_SIZE)
#define INIT_TASK_STACKSIZE           (3* configMINIMAL_STACK_SIZE)
#define MAIN_TASK_STACKSIZE           (3* configMINIMAL_STACK_SIZE)

#endif /* CONFIG_H_ */
