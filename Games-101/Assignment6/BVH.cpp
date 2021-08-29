#include "BVH.hpp"
#include <algorithm>
#include <cassert>

template<class T>
T max(T A, T B) {
	return A > B ? A : B;
}

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
	SplitMethod splitMethod)
	: maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
	primitives(std::move(p))
{
	time_t start, stop;
	time(&start);
	if (primitives.empty())
		return;

	root = recursiveBuild(primitives);

	time(&stop);
	double diff = difftime(stop, start);
	int hrs = (int)diff / 3600;
	int mins = ((int)diff / 60) - (hrs * 60);
	int secs = (int)diff - (hrs * 3600) - (mins * 60);

	printf(
		"\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
		hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
	BVHBuildNode* node = new BVHBuildNode();

	// Compute bounds of all primitives in BVH node
	Bounds3 bounds;
	for (int i = 0; i < objects.size(); ++i)
		bounds = Union(bounds, objects[i]->getBounds());
	if (objects.size() == 1)
	{
		// Create leaf _BVHBuildNode_
		node->bounds = objects[0]->getBounds();
		node->object = objects[0];
		node->left = nullptr;
		node->right = nullptr;
		return node;
	}
	else if (objects.size() == 2)
	{
		node->left = recursiveBuild(std::vector{ objects[0] });
		node->right = recursiveBuild(std::vector{ objects[1] });

		node->bounds = Union(node->left->bounds, node->right->bounds);
		return node;
	}
	else
	{
		Bounds3 centroidBounds;
		for (int i = 0; i < objects.size(); ++i)
			centroidBounds =
			Union(centroidBounds, objects[i]->getBounds().Centroid());
		int dim = centroidBounds.maxExtent();

		switch (dim)
		{
		case 0:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
				{ return f1->getBounds().Centroid().x <
				f2->getBounds().Centroid().x; });
			break;
		case 1:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
				{ return f1->getBounds().Centroid().y <
				f2->getBounds().Centroid().y; });
			break;
		case 2:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
				{ return f1->getBounds().Centroid().z <
				f2->getBounds().Centroid().z; });
			break;
		}
		if (splitMethod == SplitMethod::NAIVE){
		
			auto beginning = objects.begin();
			auto middling = objects.begin() + (objects.size() / 2);
			auto ending = objects.end();

			auto leftshapes = std::vector<Object*>(beginning, middling);
			auto rightshapes = std::vector<Object*>(middling, ending);

			assert(objects.size() == (leftshapes.size() + rightshapes.size()));

			node->left = recursiveBuild(leftshapes);
			node->right = recursiveBuild(rightshapes);
		}
		if (splitMethod == SplitMethod::SAH)
		{
			const int nBuckets = 16;
			SAHBucket buckets[nBuckets];
			//initialize buckets infomation
			for (int i = 0; i < objects.size(); i++)
			{
				Bounds3 box = objects[i]->getBounds();
				int b = nBuckets * bounds.Offset(objects[i]->getBounds().Centroid())[dim];
				if (b == nBuckets)b--;
				assert(b >= 0);
				assert(b < nBuckets);
				buckets[b].nPrimitives++;
				buckets[b].box = Union(buckets->box, box);
			}
			int temp = 0;
			for (int i = 0; i < nBuckets; i++)
				temp += buckets[i].nPrimitives;
			float time[nBuckets - 1];//COST

			for (int i = 0; i < nBuckets - 1; i++)
			{
				Bounds3 left, right;
				int leftCount = 0, rightCount = 0;
				//left part
				for (int j = 0; j <= i; j++)
				{
					left = Union(left, buckets[j].box);
					leftCount += buckets[j].nPrimitives;
				}
				//right part
				for (int j = i + 1; j < nBuckets; ++j)
				{
					right = Union(right, buckets[j].box);
					rightCount += buckets[j].nPrimitives;
				}
				time[i] = 0.125f + (left.SurfaceArea() * leftCount + right.SurfaceArea() * rightCount) / bounds.SurfaceArea();
			}
			float minTime = std::numeric_limits<float>::max();
			int minBucket = 0;
			int leftPrimitives = 0;
			for (int i = 0; i < nBuckets - 1; i++)
			{
				if (buckets[i].nPrimitives == 0)
					continue;
				if (time[i] < minTime) {
					minTime = time[i];
					minBucket = i;
				}
			}
	

			for (int i = 0; i <= minBucket; i++)
				leftPrimitives += buckets[i].nPrimitives;
			assert(leftPrimitives <= objects.size());
			assert(leftPrimitives >= 0);
			auto middling = objects.begin() + leftPrimitives;
			auto leftshapes = std::vector<Object*>(objects.begin(), middling);
			auto rightshapes = std::vector<Object*>(middling, objects.end());
			assert(objects.size() == (leftshapes.size() + rightshapes.size()));
			node->left = recursiveBuild(leftshapes);
			node->right = recursiveBuild(rightshapes);
		}
		node->bounds = Union(node->left->bounds, node->right->bounds);
	}

	return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
	Intersection isect;
	if (!root)
		return isect;
	isect = BVHAccel::getIntersection(root, ray);
	return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
	// TODO Traverse the BVH to find intersection
	Intersection inter;
	std::array<int, 3> dirIsNeg = { int(ray.direction[0] > 0), int(ray.direction[1] > 0), (int)(ray.direction[2] > 0) };
	// if (node->bounds.IntersectP(ray, Vector3f(1.0f / ray.direction.x, 1.0f / ray.direction.y, 1.0f / ray.direction.z), dirIsNeg) == false)
	//     return Intersection();
	if (node->left == nullptr && node->right == nullptr)
		return node->object->getIntersection(ray);
	Intersection inter1;
	Intersection inter2;
	if (node->left->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
		inter1 = getIntersection(node->left, ray);
	if (node->right->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
		inter2 = getIntersection(node->right, ray);
	return inter1.distance < inter2.distance ? inter1 : inter2;
}