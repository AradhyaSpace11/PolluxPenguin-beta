# PolluxPenguin

## Prerequisites
Ubuntu 20.04 LTS, ROS Noetic


### Setup

1. **Install Gazebo and Iris ArduCopter**
   - Follow the instructions in [this guide](https://github.com/monemati/multiuav-gazebo-simulation) to set up Gazebo and Iris ArduCopter.

2. **Install Dependencies**
   - Run the following command to install the required Python packages:
     ```bash
     pip install -r requirements.txt
     ```


### Running the Project

- cd into the repo core:
  ```bash
  cd PolluxPenguin-beta/core
  ```

- Ensure you have your Gemini API key ready.
- Set the key using the following command:
  ```bash
  export GOOGLE_API_KEY="your-api-key"
  ```
- Replace `'your_api_key'` with your actual Gemini API key.

- make the runner executable:
  ```bash
  chmod +x mavtest.sh
  ```
- Once the setup is complete, you can start the simulation using:
  ```bash
  ./mavtest.sh
  ```

### Additional Notes

- Ensure all prerequisites are met before running the project.
- For any issues, refer to the troubleshooting section or raise an issue on GitHub.

---
