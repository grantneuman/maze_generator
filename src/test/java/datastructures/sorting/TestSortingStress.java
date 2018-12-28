package datastructures.sorting;

import misc.BaseTest;
import misc.Searcher;

import org.junit.Test;

import datastructures.concrete.ArrayHeap;
import datastructures.concrete.DoubleLinkedList;
import datastructures.interfaces.IList;
import datastructures.interfaces.IPriorityQueue;

import static org.junit.Assert.assertTrue;
    /**
     * See spec for details on what kinds of tests this class should include.
     */        
    protected <T extends Comparable<T>> IPriorityQueue<T> makeInstance() {     // is this allowed??
        return new ArrayHeap<>();
    }
    
    @Test(timeout=10*SECOND)
    public void testInsertManyArrayHeap() {
        IPriorityQueue<String> heap = this.makeInstance();
        for (int i = 0; i < 200000; i++) {
            heap.insert("a" + i);
        }
        for (int i = 0; i < 100000; i++) {
            heap.removeMin();
        }
        assertEquals(100000, heap.size());
    }
    
    @Test(timeout=5*SECOND)
    public void testPeekAndRemoveMinManyArrayHeap() {
        IPriorityQueue<Integer> heap = this.makeInstance();
        for (int i = 200000; i >= 0; i--) {
            heap.insert(i);
        }
        
        for (int i = 0; i <= 50000; i++) {
            assertEquals(i, heap.peekMin());
            int testInt = heap.removeMin();
            assertEquals(i, testInt);
        }
    }
    
    @Test(timeout=10*SECOND)
    public void testManyTopKSortFun() {
        IList<Integer> list = new DoubleLinkedList<>();
        for (int i = 0; i < 2000000; i++) {
            list.add(i);
        }
    
        IList<Integer> top = Searcher.topKSort(5, list);
        assertEquals(5, top.size());
        for (int i = 0; i < top.size(); i++) {
            assertEquals(1999995 + i, top.get(i));
        }
    }
}
