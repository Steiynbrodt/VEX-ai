#include <iostream>
#include <fstream>  // For file handling

int main() {
    // Define the file path (can be changed to your SSD location)
    std::string filePath = "D:/log.txt";  // Modify to "D:/log.txt" (Windows) or "/mnt/ssd/log.txt" (Linux)

    // Open file in write mode
    std::ofstream logFile(filePath);

    // Check if file opened successfully
    if (!logFile) {
        std::cerr << "Error: Could not open file for writing!" << std::endl;
        return 1;
    }

    // Write some example x, y coordinates
    logFile << "X, Y\n";  // Column headers
    logFile << "100, 200\n";
    logFile << "-150, 300\n";
    logFile << "250, -400\n";
    logFile << "-350, -250\n";

    // Close the file
    logFile.close();

    std::cout << "Log file '" << filePath << "' created successfully." << std::endl;

    return 0;
}