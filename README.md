# PolluxPenguin

## Prerequisites

- **Operating System**: Ubuntu 20.04 LTS
- **ROS Version**: ROS Noetic

## Setup

1. **Install Gazebo and Iris ArduCopter**
   - Follow the instructions in [this guide](https://github.com/monemati/multiuav-gazebo-simulation) to set up Gazebo and Iris ArduCopter.

2. **Install Dependencies**
   - Run the following command to install the required Python packages:
     ```bash
     pip install -r requirements.txt
     ```

## Running the Project

1. **Navigate to the Core Directory**
   - Use the following command to move into the core directory:
     ```bash
     cd PolluxPenguin-beta/core
     ```

2. **Set the Google API Key**
   - Use the following command to set the API key:
     ```bash
     export GOOGLE_API_KEY="your-api-key"
     ```
   - Replace `"your-api-key"` with your actual Gemini API key.

3. **Make the Runner Executable**
   - Run this command to make the runner script executable:
     ```bash
     chmod +x mavtest.sh
     ```

4. **Start the Simulation**
   - Once the setup is complete, start the simulation with:
     ```bash
     ./mavtest.sh
     ```

### Additional Notes

- Ensure all prerequisites are met before running the project.
- For any issues, refer to the troubleshooting section or raise an issue on GitHub.

---
