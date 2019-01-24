#include "chBroadPhase.h"


Chan::CGBroadPhase::CGBroadPhase()
{
	m_proxyCount = 0;

	m_pairCapacity = 16;
	m_pairCount = 0;
	m_pairBuffer = new cgPair[m_pairCapacity];

	m_moveCapacity = 16;
	m_moveCount = 0;
	m_moveBuffer = new int[m_moveCapacity];
}

Chan::CGBroadPhase::~CGBroadPhase()
{
	delete[] m_moveBuffer;
	delete[] m_pairBuffer;
	m_moveBuffer = nullptr;
	m_pairBuffer = nullptr;
}

int Chan::CGBroadPhase::CreateProxy(const c2AABB & aabb, void * userData)
{
	int proxyId = m_tree.CreateProxy(aabb, userData);
	++m_proxyCount;
	BufferMove(proxyId);
	return proxyId;
}

void Chan::CGBroadPhase::DestroyProxy(int proxyId)
{
	UnBufferMove(proxyId);
	--m_proxyCount;
	m_tree.DestroyProxy(proxyId);
}

void Chan::CGBroadPhase::UpdateProxy(int proxyId, const c2AABB & aabb, const ChVector2 & displacement)
{
	bool buffer = m_tree.UpdateProxy(proxyId, aabb, displacement);
	if (buffer)
		BufferMove(proxyId);
}

void Chan::CGBroadPhase::UpdateProxy(int proxyId, const c2AABB & aabb)
{
	bool buffer = m_tree.UpdateProxy(proxyId, aabb);
	if (buffer)
		BufferMove(proxyId);
}

// This is called from DynamicTrees::query when we are gathering pairs
bool Chan::CGBroadPhase::QueryCallback(int proxyId)
{
	// A proxy cannot form a pair with itself
	if (proxyId == m_queryProxyId)
		return true;

	// Grow the pair buffer as needed.
	if (m_pairCount == m_pairCapacity)
	{
		cgPair* oldBuffer = m_pairBuffer;
		m_pairCapacity *= 2;
		m_pairBuffer = new cgPair[m_pairCapacity];
		memcpy(m_pairBuffer, oldBuffer, m_pairCount * sizeof(cgPair));
		delete[] oldBuffer;
		oldBuffer = nullptr;
	}

	m_pairBuffer[m_pairCount].proxyIdA = (int)Min((ChReal)proxyId, (ChReal)m_queryProxyId);
	m_pairBuffer[m_pairCount].proxyIdB = (int)Max((ChReal)proxyId, (ChReal)m_queryProxyId);
	++m_pairCount;

	return true;
}

void Chan::CGBroadPhase::BufferMove(int proxyId)
{
	if (m_moveCount == m_moveCapacity)
	{
		int* oldBuffer = m_moveBuffer;
		m_moveCapacity *= 2;
		m_moveBuffer = new int[m_moveCapacity];
		memcpy(m_moveBuffer, oldBuffer, m_moveCount * sizeof(int));
		delete[] oldBuffer;
		oldBuffer = nullptr;
	}

	m_moveBuffer[m_moveCount] = proxyId;
	++m_moveCount;
}

void Chan::CGBroadPhase::UnBufferMove(int proxyId)
{
	for (int i = 0; i < m_moveCount; ++i)
	{
		if (m_moveBuffer[i] == proxyId)
			m_moveBuffer[i] = e_nullProxy;
	}
}


void * Chan::CGBroadPhase::GetUserData(int proxyId) const
{
	return m_tree.GetUserData(proxyId);
}

bool Chan::CGBroadPhase::TestOverlap(int proxyIdA, int proxyIdB)
{
	const c2AABB& aabbA = m_tree.GetFatAABB(proxyIdA);
	const c2AABB& aabbB = m_tree.GetFatAABB(proxyIdB);
	return aabbOverlap(aabbA, aabbB);
}

int Chan::CGBroadPhase::GetProxyCount() const
{
	return m_proxyCount;
}
