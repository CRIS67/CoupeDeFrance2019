#include "trajectoryHandle.hpp"

/*
Detects where the angles of the trajectory change and thus 
simplifies the trajectory 
*/
std::vector<Node> pathTreatment(std::vector<Node> path)
{	
	std::vector<Node> simplifiedPath;

	for(uint i = 0; i<path.size()-2; i++)
	{
		Node tmpNode = path.at(i); 
		Node nextNode = path.at(i+1); 
		Node furtherNode = path.at(i+2); 

		int xVector = nextNode.coord.first - tmpNode.coord.first; 
		int yVector = nextNode.coord.second - tmpNode.coord.second; 

		int xVectorF = furtherNode.coord.first -  nextNode.coord.first ; 
		int yVectorF = furtherNode.coord.second - nextNode.coord.second; 

		double firstAngle = atan2(yVector,xVector); 
		double secondAngle = atan2(yVectorF, xVectorF); 
		if(firstAngle != secondAngle)
		{
			simplifiedPath.push_back(nextNode); 
		}

	}

	return simplifiedPath; 
}

bool sensorTreatment(int enemyX, int enemyY, int enemyWidth, 
	std::vector<std::vector<int>>& mapVector, std::vector<Node> path)
{
	clearMap(mapVector); // clears the map 
	generateMap(mapVector, mapVector.size() , mapVector.at(0).size()); // initializes the original map 
	//createRectangle( enemyX, enemyY, enemyWidth, enemyWidth, mapVector); // generates the obstacle zone  

	// Check if the path passes through the obsctale 
	for(uint i = 0; i< path.size(); i++)
	{
		int x = path.at(i).coord.first; 
		int y = path.at(i).coord.second; 

		if(mapVector[x][y] == 1)
		{
			return true; 
		}
	}

	return false; 
}

void printPath(std::vector<Node> path, std::vector<std::vector<int>>& mapVector)
{

    std::cout <<"===== PRINTING PATH =====" << std::endl << std::endl; 
    std::vector<std::vector<int>> tmpMap = mapVector; 

    int x,y; 
    for(uint i = 0; i< path.size(); i++)
    {
    x = path.at(i).coord.first;
    y = path.at(i).coord.second;

    tmpMap[x][y] = 2;
    }

    printMap(tmpMap.size(), tmpMap[0].size(), tmpMap);
}

bool detectCollision(std::vector<std::vector<int> > &map, std::vector<Node> path){
	for(unsigned int i = 0; i < path.size() -1; i++){
		/*int xA = path.at(i).coord.first;
		int yA = path.at(i).coord.second;
		int xB = path.at(i).coord.first;
		int yB = path.at(i).coord.second;*/
		std::cout << path.at(i).coord.first << " & " << path.at(i).coord.second << " -> " <<  path.at(i+1).coord.first << " & " << path.at(i+1).coord.second << std::endl;
		if(detectCollisionLine(path.at(i).coord.first, path.at(i).coord.second, path.at(i+1).coord.first, path.at(i+1).coord.second,map))
            return true;

		//std::cout << "draw" << std::endl;
        //drawLine(path.at(i).coord.first, path.at(i).coord.second, path.at(i+1).coord.first, path.at(i+1).coord.second,map);
	}
	return false;
}
void optimizePath(std::vector<std::vector<int> > &map, std::vector<Node> path){
    unsigned int lastIndex = path.size()-1;
	
	std::vector<Node> newPath;

	/*
    std::vector<int> vX2;
    std::vector<int> vY2;*/

    unsigned int ind = 0;
    
	int x = path.at(ind).coord.first;
    int y = path.at(ind).coord.second;
	/*
    vX2.push_back(x);
    vY2.push_back(y);*/
	
	Node n = path.at(ind);
	newPath.push_back(n);

    while(ind != lastIndex){
        bool shortcut = false;
        for(unsigned int i = lastIndex; i > ind+1;i--){
            if(!detectCollisionLine(x,y,path.at(i).coord.first,path.at(i).coord.second,map)){
                shortcut = true;
                ind = i;
				/*
                x = path.at(ind).coord.first
                y = path.at(ind).coord.second
				
                vX2.push_back(x);
                vY2.push_back(y);*/
				
				n = path.at(ind);
				newPath.push_back(n);
				
                break;
            }
        }
        if(!shortcut){
            ind++;
			/*
            x = path.at(ind).coord.first
            y = path.at(ind).coord.second
            vX2.push_back(x);
            vY2.push_back(y);*/
			
			n = path.at(ind);
			newPath.push_back(n);
        }
    }
	/*
    for(unsigned int i = 0; i < vX2.size()-1;i++){
        std::cout << vX2.at(i) << " & " << vY2.at(i) << " -> " <<  vX2.at(i+1) << " & " << vY2.at(i+1) << std::endl;
        drawLine(vX2.at(i),vY2.at(i),vX2.at(i+1),vY2.at(i+1),map);
    }*/


}
bool detectCollisionLine( float x1, float y1, float x2, float y2 , std::vector<std::vector<int> >& mapVector){
        // Bresenham's line algorithm
  const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
  if(steep)
  {
    std::swap(x1, y1);
    std::swap(x2, y2);
  }

  if(x1 > x2)
  {
    std::swap(x1, x2);
    std::swap(y1, y2);
  }

  const float dx = x2 - x1;
  const float dy = fabs(y2 - y1);

  float error = dx / 2.0f;
  const int ystep = (y1 < y2) ? 1 : -1;
  int y = (int)y1;

  const int maxX = (int)x2;

  for(int x=(int)x1; x<=maxX; x++)
  {
    //std::cout << x << " / " << y << std::endl;
    if(steep)
    {
        if(mapVector.at(y).at(x) == 1)
            return true;
    }
    else
    {
        if(mapVector.at(x).at(y) == 1)
            return true;
    }

    error -= dy;
    if(error < 0)
    {
        y += ystep;
        error += dx;
    }
  }
  return false;
}