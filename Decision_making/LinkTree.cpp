#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Define a structure for the linked list node
typedef struct DeviceNode {
    char id[5]; // Store device ID (4 characters + null terminator)
    struct DeviceNode* next;
} DeviceNode;

// Function to create a new node
DeviceNode* createNode(const char* id) {
    DeviceNode* newNode = (DeviceNode*)malloc(sizeof(DeviceNode));
    if (!newNode) {
        printf("Memory allocation failed!\n");
        return NULL;
    }
    strncpy(newNode->id, id, 4);
    newNode->id[4] = '\0'; // Ensure null termination
    newNode->next = NULL;
    return newNode;
}

// Function to add a device to the list
void addDevice(DeviceNode** head, const char* id) {
    if (!head) return;
    
    // Check if the device already exists
    DeviceNode* current = *head;
    while (current) {
        if (strncmp(current->id, id, 4) == 0) {
            printf("Device %s already exists in the list.\n", id);
            return;
        }
        current = current->next;
    }
    
    // Create and insert new device
    DeviceNode* newNode = createNode(id);
    if (!newNode) return;
    
    newNode->next = *head;
    *head = newNode;
    printf("Device %s added successfully.\n", id);
}

// Function to print the list
void printDevices(DeviceNode* head) {
    printf("Registered Devices:\n");
    while (head) {
        printf("%s\n", head->id);
        head = head->next;
    }
}

// Function to free memory
void freeDevices(DeviceNode* head) {
    DeviceNode* temp;
    while (head) {
        temp = head;
        head = head->next;
        free(temp);
    }
}

int main() {
    DeviceNode* deviceList = NULL;
    
    // Adding devices dynamically from received messages
    addDevice(&deviceList, "WAVE");
    addDevice(&deviceList, "VEWA");
    addDevice(&deviceList, "NEW1");
    addDevice(&deviceList, "VEWA"); // Should not add duplicate
    
    // Print all registered devices
    printDevices(deviceList);
    
    // Free allocated memory
    freeDevices(deviceList);
    
    return 0;
}
