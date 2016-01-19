bool onSegment(pair<double,double> p,pair<double,double> q,pair<double,double> r)
{
    if (q.first <= max(p.first, r.first) && q.first >= min(p.first, r.first) &&
        q.second <= max(p.second, r.second) && q.second >= min(p.second, r.second))
       return true;
 
    return false;
}
int orientation(pair<double,double> p,pair<double,double> q,pair<double,double> r)
{
	int val = (q.second - p.second) * (r.first - q.first) -
              (q.first - p.first) * (r.second - q.second); 
    if (val == 0) return 0;  // colinear 
    return (val > 0)? 1: 2;  // clock or counterclock wise
}
bool doIntersect(pair<double,double> segments1[][2],pair<double,double> segments2[][2],int n1,int n2)
{
	for(int i=0;i<n1;i++)
	for(int j=0;j<n2;j++)
	{
		int o1=orientation(segments1[i][0],segments1[i][1],segments2[j][0]);
		int o2=orientation(segments1[i][0],segments1[i][1],segments2[j][1]);
		int o3=orientation(segments2[j][0],segments2[j][1],segments1[i][0]);
		int o4=orientation(segments2[j][0],segments2[j][1],segments1[i][1]);
		if(o1!=o2 && o3!=o4)
			return true;
		//special case, on segment scenario
		if(o1==0 && onSegment(segments1[i][0],segments2[j][0],segments1[i][1])) return true;
		if(o2==0 && onSegment(segments1[i][0],segments2[j][1],segments1[i][1])) return true;
		if(o3==0 && onSegment(segments2[j][0],segments1[i][0],segments2[j][1])) return true;
		if(o4==0 && onSegment(segments2[j][0],segments1[i][1],segments2[j][1])) return true;
	}
	return false;
}
bool isFeasible_static(pair<int,int> tworobots,int type[])
{
	// Check for overlap of two robots by checking edge intersection
	int n_edges1=2*DIM1;
	int n_edges2=2*DIM2;
	int r1=tworobots.first,r2=tworobots.second;
	pair<double,double> r1_edges[n_edges1][2];	//each pair is of the form (startpoint,endpoint) for a line-segment
	pair<double,double> r2_edges[n_edges2][2];
	for(int i=0;i<n_edges1;i++)
	{
		r1_edges[i][0]=edges1[r1*n_edges1+i][0];
		r1_edges[i][1]=edges1[r1*n_edges1+i][1];
	}  
	 
	for(int i=0;i<n_edges2;i++)
	{
		r2_edges[i][0]=edges2[r2*n_edges2+i][0];
		r2_edges[i][1]=edges2[r2*n_edges2+i][1];	
	}
	return !doIntersect(r1_edges,r2_edges,n_edges1,n_edges2);
}
bool isFeasible_static_multi(vector<int> robots,bool mask[],int type[])
{
	for(int i=0;i<robots.size()-1;i++){
		for(int j=i+1;j<robots.size();j++){
			if(mask[i] || mask[j])continue;
			string temp=to_string(robots[i])+"-"+to_string(robots[j]);
			if(doCollide_static[i][j].find(temp)!=doCollide_static[i][j].end()){
				if(doCollide_static[i][j][temp])
					return false;
				continue;
			}
			int category[2];
			category[0]=type[i];category[1]=type[j];
			doCollide_static[i][j][temp]=!isFeasible_static(make_pair(robots[i],robots[j]),category);
			if(doCollide_static[i][j][temp])
				return false;
		}
	}
	return true;
}
bool isFeasible_dynamic(pair<int,int> tworobots_from,pair<int,int> tworobots_to,int type[])
{
	// Check for collision between two robots while in transition
	int r1_from=tworobots_from.first,r2_from=tworobots_from.second;
	int r1_to=tworobots_to.first,r2_to=tworobots_to.second;
	// 1)Edges
	int n_edges1=2*DIM1;
	int n_edges2=2*DIM2;
	pair<double,double> r1_from_edges[n_edges1][2],r2_from_edges[n_edges2][2],r1_to_edges[n_edges1][2],r2_to_edges[n_edges2][2];
	for(int i=0;i<n_edges1;i++)
	{
		r1_from_edges[i][0]=edges1[r1_from*n_edges1+i][0];
		r1_from_edges[i][1]=edges1[r1_from*n_edges1+i][1];
		r1_to_edges[i][0]=edges1[r1_to*n_edges1+i][0];
		r1_to_edges[i][1]=edges1[r1_to*n_edges1+i][1];
	}
	for(int i=0;i<n_edges2;i++)
	{
		r2_from_edges[i][0]=edges2[r2_from*n_edges2+i][0];
		r2_from_edges[i][1]=edges2[r2_from*n_edges2+i][1];
		r2_to_edges[i][0]=edges2[r2_to*n_edges2+i][0];
		r2_to_edges[i][1]=edges2[r2_to*n_edges2+i][1];
	}
	// 2)Tracks
	int n_trackpt1=trackpt1[0].size();
	int n_trackpt2=trackpt2[0].size();
	pair<double,double> r1_tracks[n_trackpt1][2];
	pair<double,double> r2_tracks[n_trackpt2][2];
	if(tworobots_from.first!=tworobots_to.first){
		for(int i=0;i<n_trackpt1;i++){
			r1_tracks[i][0]=trackpt1[r1_from][i];
			r1_tracks[i][1]=trackpt1[r1_to][i];
		}
	}
	if(tworobots_from.second!=tworobots_to.second){
		for(int i=0;i<n_trackpt2;i++){
			r2_tracks[i][0]=trackpt2[r2_from][i];
			r2_tracks[i][1]=trackpt2[r2_to][i];
		}
	}
	// Detecting Track-track collision
	if(tworobots_from.first!=tworobots_to.first && tworobots_from.second!=tworobots_to.second){
		if(doIntersect(r1_tracks,r2_tracks,n_trackpt1,n_trackpt2))return false;
	}
	// Detecting Track-Edge collison
	if(tworobots_from.first!=tworobots_to.first){
		if(doIntersect(r1_tracks,r2_from_edges,n_trackpt1,n_edges2))return false;
		if(tworobots_from.second!=tworobots_to.second)
		if(doIntersect(r1_tracks,r2_to_edges,n_trackpt1,n_edges2))return false;
	}
	if(tworobots_from.second!=tworobots_to.second){
		if(doIntersect(r1_from_edges,r2_tracks,n_edges1,n_trackpt2))return false;
		if(tworobots_from.first!=tworobots_to.first)
		if(doIntersect(r1_to_edges,r2_tracks,n_edges1,n_trackpt2))return false;
	}
	return true;
}
bool isFeasible_dynamic_multi(vector<int> robots_from,vector<int> robots_to,bool mask[],int type[])
{
	for(int i=0;i<robots_from.size()-1;i++){
		for(int j=i+1;j<robots_from.size();j++){
			if(mask[i] || mask[j])continue;
			string temp=to_string(robots_from[i])+"-"+to_string(robots_from[j])+"-"+to_string(robots_to[i])+"-"+to_string(robots_to[j]);
			if(doCollide_dynamic[i][j].find(temp)!=doCollide_dynamic[i][j].end()){
				if(doCollide_dynamic[i][j][temp])
					return false;
				continue;
			}
			int category[2];
			category[0]=type[i];category[1]=type[j];
			doCollide_dynamic[i][j][temp]=!isFeasible_dynamic(make_pair(robots_from[i],robots_from[j]),make_pair(robots_to[i],robots_to[j]),category);
			if(doCollide_dynamic[i][j][temp])
				return false;
		}
	}
	return true;
}