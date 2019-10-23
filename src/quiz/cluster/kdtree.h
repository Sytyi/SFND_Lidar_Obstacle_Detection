/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

const uint N_DIMENSIONS = 3;
// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
    {
	    delete left;
	    delete right;
    }
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
	    delete root;
	}

    void insertHelper(Node<PointT> *&node, const PointT & point, int id, uint level)
    {
        if(node == NULL)
        {
            node = new Node<PointT>(point, id);
        }
        else
        {
            Node<PointT> *& next = point.data[level%N_DIMENSIONS]
                    < node->point.data[level%N_DIMENSIONS] ? node->left : node->right;
            insertHelper( next, point, id, level+1);
        }

    }

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        insertHelper(root,point, id, 0);
	}

	bool inBBox(const PointT &target, const PointT &point, float distanceTol)
    {
	    for(int i = 0; i < N_DIMENSIONS; ++i)
        {
            if( fabs(point.data[i] - target.data[i]) > distanceTol )
                return false;
        }
	    return true;
    }

    float distance(const PointT &a, const PointT &b)
    {
	    float sqrD = 0;
        for(int i = 0; i < N_DIMENSIONS; ++i)
        {
            float d = a.data[i] - b.data[i];
            sqrD += d*d;
        }

        return sqrt(sqrD);
    }


	void searchHelper(Node<PointT> * node, std::vector<int> & ids, const PointT & target, float distanceTol, uint depth)
    {
	    if( !node )
	        return;

	    if( inBBox(target, node->point, distanceTol) )
            if (distance(target, node->point) < distanceTol)
                ids.push_back(node->id);

	    if ( target.data[depth % N_DIMENSIONS] - distanceTol < node->point.data[depth % N_DIMENSIONS] )
	        searchHelper(node->left, ids, target, distanceTol, depth + 1);

        if ( target.data[depth % N_DIMENSIONS] + distanceTol > node->point.data[depth % N_DIMENSIONS] )
            searchHelper(node->right, ids, target, distanceTol, depth + 1);

    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, ids, target, distanceTol, 0);
		return ids;
	}
	

};


template<typename PointT>
void proximity(int id, std::vector<int> & cluster, std::vector<bool> &processed,
               const typename pcl::PointCloud<PointT>::Ptr& points, KdTree<PointT>& tree, float distanceTol)
{
    processed[id] = true;
    cluster.push_back(id);
    auto neighbors = tree.search(points->at(id), distanceTol);
    for(int p : neighbors)
    {
        if( ! processed[p])
            proximity(p, cluster, processed, points, tree, distanceTol);
    }
}

template<typename PointT>
std::vector<std::vector<int>> euclideanCluster(const typename pcl::PointCloud<PointT>::Ptr& cloud, KdTree<PointT>& tree,
                                               float distanceTol)
{
    std::vector<std::vector<int>> clusters;

    std::vector<bool> processed;
    processed.assign(cloud->points.size(), false);
    for( int i = 0; i < cloud->points.size(); ++i)
    {
        if (! processed[i])
        {
            std::vector<int> cluster;
            proximity(i, cluster, processed, cloud, tree, distanceTol );
            clusters.push_back(cluster);
        }
    }
    return clusters;
}



