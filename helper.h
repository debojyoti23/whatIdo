void showtime()
{
	time_t t=time(0);
	struct tm *now=localtime(&t);
	cout << (now->tm_hour) << ':' 
         << (now->tm_min) << ':'
         <<  now->tm_sec
         << endl;
}
inline string formKey(vector<int> robots)
{
	string temp=to_string(robots[0]);
	for(int i=1;i<robots.size();i++)
		temp=temp+"-"+to_string(robots[i]);
	return temp;
}
inline string formKey_indexed(vector<int> robots,vector<int> positions)
{
	string temp=to_string(robots[0]);
	string temp1=to_string(positions[0]);
	for(int i=1;i<robots.size();i++){
		temp=temp+"-"+to_string(robots[i]);
		temp1=temp1+"-"+to_string(positions[i]);
	}
	temp=temp+":"+temp1;
	return temp;
}
bool isless(vector<int> v1,vector<int> v2)
{
	// check if v1<=v2
	for(int i=0;i<v1.size();i++){
		if(v1[i]>v2[i])
			return false;
	}
	return true;
}