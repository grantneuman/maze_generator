package datastructures.sorting;

import static org.junit.Assert.assertTrue;

import misc.BaseTest;
import misc.exceptions.EmptyContainerException;
import datastructures.concrete.ArrayHeap;
import datastructures.interfaces.IPriorityQueue;
import org.junit.Test;

/**
 * See spec for details on what kinds of tests this class should include.
 */
public class TestArrayHeapFunctionality extends BaseTest {
    protected <T extends Comparable<T>> IPriorityQueue<T> makeInstance() {
        return new ArrayHeap<>();
    }

    @Test(timeout=SECOND)
    public void testBasicSize() {
        IPriorityQueue<Integer> heap = this.makeInstance();
        heap.insert(3);
        assertEquals(1, heap.size());
        assertTrue(!heap.isEmpty());
    }
    
    @Test(timeout=SECOND)
    public void testInsertBasic() {
        IPriorityQueue<Integer> heap = this.makeInstance();
        heap.insert(3);
        assertEquals(1, heap.size());
        assertTrue(!heap.isEmpty());
        assertEquals(3, heap.removeMin());
        heap.insert(10);
        assertEquals(10, heap.removeMin());
        heap.insert(5);
        heap.insert(2);
        heap.insert(50);
        assertEquals(2, heap.removeMin());
    }

    @Test(timeout=SECOND)
    public void testInsertDuplicate() {
        IPriorityQueue<Integer> heap = this.makeInstance();
        heap.insert(3);
        heap.insert(3);
        heap.insert(5);
        assertEquals(3, heap.removeMin());
        assertEquals(3, heap.removeMin());
    }
    
    @Test(timeout=SECOND)
    public void testRemoveMin() {
        IPriorityQueue<Integer> heap = this.makeInstance();
        for (int i = 1; i <= 50; i++) {
            heap.insert(i);
        }
        assertEquals(1, heap.removeMin());
        assertEquals(2, heap.removeMin());
        int holder = 3;
        for (int i = 1; i <= 48; i++) {
            holder += heap.removeMin();
        }
        assertEquals(1275, holder);
        assertEquals(0, heap.size());
    }

    @Test(timeout=SECOND)
    public void addAndRemoveMin() {
        IPriorityQueue<Integer> heap = this.makeInstance();
        for (int i = 1; i <= 20; i++) {
            heap.insert(i);
        }
        for (int i = 1; i <= 10; i++) {
            heap.removeMin();
        }
        for (int i = 1; i <= 10; i++) {
            heap.insert(i);
        }
        for (int i = 1; i <= 20; i++) {
            assertEquals(i, heap.removeMin());
        }
    }
    
    @Test(timeout=SECOND)
    public void testPeekMin() {
        IPriorityQueue<Integer> heap = this.makeInstance();
        for (int i = 20; i >= 1; i--) {
            heap.insert(i*i);
        }
        assertEquals(20, heap.size());
        assertEquals(1, heap.peekMin());
        int one = heap.removeMin();
        assertEquals(1, one);
        int two = heap.removeMin();
        assertEquals(4, two);
        int three = heap.removeMin();
        assertEquals(9, three);
        assertEquals(16, heap.peekMin()); 
        int four = heap.removeMin(); 
        assertEquals(16, four);
        assertEquals(25, heap.peekMin());
        int five = heap.removeMin(); 
        assertEquals(25, five);
        assertEquals(36, heap.peekMin());
        int six = heap.removeMin(); 
        assertEquals(36, six);
        assertEquals(49, heap.peekMin());
        int seven = heap.removeMin(); 
        assertEquals(49, seven);
        assertEquals(64, heap.peekMin());
    }

    @Test(timeout=SECOND)
    public void testPeekMinNonRemoval() {
        IPriorityQueue<Integer> heap = this.makeInstance();
        heap.insert(19);
        heap.insert(20);
        heap.insert(1);
        assertEquals(1, heap.peekMin());
        heap.peekMin();
        assertEquals(3, heap.size());
        heap.peekMin();
        assertEquals(3, heap.size());
    }

    @Test(timeout=SECOND)
    public void testPeekMinRepeated() {
        IPriorityQueue<Integer> heap = this.makeInstance();
        heap.insert(19);
        heap.insert(20);
        heap.insert(1);
        assertEquals(1, heap.peekMin());
        assertEquals(1, heap.peekMin());
        assertEquals(1, heap.peekMin());
    }

    @Test(timeout=SECOND)
    public void testGenerics() {
        IPriorityQueue<String> heap = this.makeInstance();
        heap.insert("b");
        heap.insert("a");
        heap.insert("z");
        heap.insert("apple");
        assertEquals("a", heap.peekMin());
        String min = heap.removeMin();
        assertEquals("a", min);
        String secondMin = heap.removeMin();
        assertEquals("apple", secondMin);
    }

    @Test(expected = EmptyContainerException.class)
    public void testExceptionRemoveMin() {
        IPriorityQueue<Integer> heap = this.makeInstance();
        heap.insert(1);
        heap.removeMin();
        heap.removeMin();
    }
    
    @Test(expected = EmptyContainerException.class)
    public void testExceptionPeekMin() {
        IPriorityQueue<Integer> heap = this.makeInstance();
        heap.insert(1);
        heap.removeMin();
        heap.peekMin();
    }
    
    @Test(expected = IllegalArgumentException.class)
    public void testExceptionInsert() {
        IPriorityQueue<Integer> heap = this.makeInstance();
        heap.insert(null);
    }
}
