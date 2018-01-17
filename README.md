# API-Motion<
 
This is a sample project containing the macro motion profiling system developed in the 2017 frc season. This project can be used as a base to create software for following seasons while leaving autonomous programming already complete. 

#
#### Requirements</h4>
- A Java development eviroment
- Java JDK 8
#
#### Setup
1. Clone the project to your directory of choice
2. Open a terminal and run either 

		./gradlew idea
		
	or
	
		./gradlew eclipse
		
	to setup the project for you favorite IDE
	
3. while connected to the internet run 
	
		./gradlew build deploy
		
	this will download the required dependencies.
	
4. then connect to the robot and run

		./gradle build deploy --offline
		
	to upload code while connected to your robot