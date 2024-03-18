#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "comms.h"
#include "info.c"
#include "tusb.h" // TinyUSB header

// Function declarations
void printMenu();
void handleOption1(Motor *motors);
void handleOption2();
void handleOption3();
void exitProgram();

int menu(Motor *motors) {
    int choice;

    
    // Print menu and get user choice
    
    
    while(tud_cdc_available() == 0) { // Wait for data to be available
        // Do nothing
        printf("Enter your choice: ");
        printMenu();
        printf("motorTest %s\n" , motors[0].name);
        sleep_ms(3000);
    }
    if (tud_cdc_available()) { // Check if data is available to read
        uint8_t buf[64];
        uint32_t count = tud_cdc_read(buf, sizeof(buf)); // Read the data into buf
        
        choice = atoi(buf); // Convert the data to an integer
        // Perform action based on the choice
        switch(choice) {
            case 1:
                printf("echo from 1 %s\n", buf); // Print the data
                handleOption1( motors);
            
                break;
            case 2:
                printf("echo from 2 %s\n", buf); // Print the data
                //handleOption2();
                break;
            case 3:
                printf("echo from 3 %s\n", buf); // Print the data
                //handleOption3();
                break;
            case 4:
            // exitProgram();
                //return 0; // Exit the program
                break;
            default:
                printf("echo %s\n", buf); // Print the data
                printf("choice is %d\n", choice);
                //printf("Invalid choice. Please try again.\n");
        }

    }
    return 0;
}

// Function to print the menu
void printMenu() {
    printf("\n=== MENU ===\n");
    printf("1. Option 1\n");
    printf("2. Option 2\n");
    printf("3. Option 3\n");
    printf("4. Exit\n");
}

// Function for handling option 1
void handleOption1(Motor *motors) {
    printf("Option 1 selected. getting motor info\n");
    char motor_info[64];
    while(1)
    {
        if(tud_cdc_available()) { // Check if data is available to read
            tud_cdc_write_flush();
            break;
        }
        if(motors[0].motorStats.velocity!= 0.0)
        {   
            get_motor_info(&motors[0],motor_info);
            printf("Stats of: %s, \n",motor_info);
        }
        if(motors[1].motorStats.velocity!= 0.0)
        {
            get_motor_info(&motors[1],motor_info);
            printf("Stats of: %s, \n", motor_info);
        }

        
    }

    // Implement the action for option 1
}

// Function for handling option 2
void handleOption2() {
    printf("Option 2 selected.\n");
    // Implement the action for option 2
}

// Function for handling option 3
void handleOption3() {
    printf("Option 3 selected.\n");
    // Implement the action for option 3
}

// Function to exit the program
void exitProgram() {
    printf("Exiting program.\n");
}