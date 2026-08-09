#include <unitree/robot/client/client.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }
  //Initialize interface
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  
  //Instantiate sport_client and initialize
  unitree::robot::go2::SportClient sport_client;
  sport_client.SetTimeout(10.0f);//Timeout duration
  sport_client.Init();
  sport_client.WaitLeaseApplied();
  int moveTime = 3; // move 3 second
  auto startTime = std::chrono::steady_clock::now();
  while(true)
  { 
    
    sport_client.RecoveryStand(); //Recovery stand-up
    sleep(3);
    sport_client.Sit(); //Sit down
    sleep(3);
    sport_client.RiseSit(); //Recover from sitting position
    sleep(1);
    sport_client.Move(0.2,0,0); //Set the desired velocity in the X direction to control the Go2 robot to walk forward for a certain distance
    sleep(2); //Walk forward for 2 seconds
    
    // Get current time point
    auto currentTime = std::chrono::steady_clock::now();
    // Calculate elapsed time
    auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);
    // Exit loop if elapsed time is greater than or equal to 3 seconds (Note: The variable is moveTime=3, not 2 as in the comment above)
    if (elapsedTime.count() >= moveTime) 
    {
        break;
    }
  }
  
    sleep(2); 
    return 0;
}
