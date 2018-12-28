package datastructures.sorting;

import misc.BaseTest;
import datastructures.concrete.DoubleLinkedList;
import datastructures.interfaces.IList;
import misc.Searcher;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.junit.Test;

/**
 * See spec for details on what kinds of tests this class should include.
 */
public class TestTopKSortFunctionality extends BaseTest {
    @Test(timeout=SECOND)
    public void testSimpleUsage() {
        IList<Integer> list = new DoubleLinkedList<>();
        for (int i = 0; i < 20; i++) {
            list.add(i);
        }

        IList<Integer> top = Searcher.topKSort(5, list);
        assertEquals(5, top.size());
        for (int i = 0; i < top.size(); i++) {
            assertEquals(15 + i, top.get(i));
        }
    }
    
    @Test(timeout=SECOND)
    public void testTopKSortBasic() {
        IList<Integer> list = new DoubleLinkedList<>();
        for (int i = 0; i < 80; i++) {
            list.add(20 - i % 20);
        }
        
        List<Integer> sortedList = new ArrayList<Integer>();
        for (int element : list) {
            sortedList.add(element);
        }
        Collections.sort(sortedList);
        List<Integer> sortedIList = new ArrayList<Integer>();
        for (int element : sortedList) {
            sortedIList.add(element);
        }
        
        IList<Integer> topKSortedList = Searcher.topKSort(5, list);
        for (int i = 0; i < topKSortedList.size(); i++) {
            assertEquals(sortedIList.get(sortedIList.size() - 5 + i), topKSortedList.get(i));
        }
    }
    
    @Test(timeout=SECOND)
    public void testSmallList() {
        IList<Integer> list = new DoubleLinkedList<>();
        for (int i = 2; i >= 0; i--) {
            list.add(i);
        }
        list.add(5);
        
        List<Integer> sortedList = new ArrayList<Integer>();
        for (int element : list) {
            sortedList.add(element);
        }
        Collections.sort(sortedList);
        List<Integer> sortedIList = new ArrayList<Integer>();
        for (int element : sortedList) {
            sortedIList.add(element);
        }
        
        IList<Integer> topKSortedList = Searcher.topKSort(5, list);
        for (int i = 0; i < sortedIList.size(); i++) {
            assertEquals(sortedIList.get(i), topKSortedList.get(i));
        }
    }
    
    @Test(timeout=SECOND)
    public void testListUnmodified() {
        IList<Integer> list = new DoubleLinkedList<>();
        for (int i = 0; i < 80; i++) {
            list.add(20 - i % 20);
        }
        
        List<Integer> sortedList = new ArrayList<Integer>();
        for (int element : list) {
            sortedList.add(element);
        }
        Collections.sort(sortedList);
        List<Integer> sortedIList = new ArrayList<Integer>();
        for (int element : sortedList) {
            sortedIList.add(element);
        }
        
        IList<Integer> topKSortedList = Searcher.topKSort(5, list);
        for (int i = 0; i < list.size(); i++) {
            assertEquals(20 - i % 20, list.get(i));
        }
    }
    
    @Test(timeout=SECOND)
    public void testOneK() {
        IList<Integer> list = new DoubleLinkedList<>();
        for (int i = 0; i < 80; i++) {
            list.add(20 - i % 20);
        }
        
        IList<Integer> topKSortedList = Searcher.topKSort(1, list);
        assertEquals(1, topKSortedList.size());
        assertEquals(20, topKSortedList.get(0));
    }
    
    @Test(timeout=SECOND)
    public void testOneKOneElement() {
        IList<Integer> list = new DoubleLinkedList<>();
        list.add(20);
        
        IList<Integer> topKSortedList = Searcher.topKSort(1, list);
        assertEquals(1, topKSortedList.size());
        assertEquals(20, topKSortedList.get(0));
    }
    
    @Test(timeout=SECOND)
    public void testKEqualsSize() {
        IList<Integer> list = new DoubleLinkedList<>();
        list.add(20);
        list.add(15);
        list.add(25);
        list.add(17);
        list.add(15);
        
        IList<Integer> topKSortedList = Searcher.topKSort(5, list);
        assertEquals(5, topKSortedList.size());
        assertEquals(15, topKSortedList.get(0));
        assertEquals(15, topKSortedList.get(1));
        assertEquals(17, topKSortedList.get(2));
        assertEquals(20, topKSortedList.get(3));
        assertEquals(25, topKSortedList.get(4));
    }
    
    @Test(timeout=SECOND)
    public void testZeroK() {
        IList<Integer> list = new DoubleLinkedList<>();
        for (int i = 0; i < 80; i++) {
            list.add(20 - i % 20);
        }
        
        IList<Integer> topKSortedList = Searcher.topKSort(0, list);
        assertEquals(0, topKSortedList.size());
    }
    
    @Test(expected = IllegalArgumentException.class)
    public void testExceptionNegativeK() {
        IList<Integer> list = new DoubleLinkedList<>();
        for (int i = 0; i < 80; i++) {
            list.add(20 - i % 20);
        }
        
        IList<Integer> topKSortedList = Searcher.topKSort(-1, list);
    }
}
