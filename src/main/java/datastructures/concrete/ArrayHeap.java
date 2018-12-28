package datastructures.concrete;

import datastructures.interfaces.IPriorityQueue;
import misc.exceptions.EmptyContainerException;

/**
 * See IPriorityQueue for details on what each method must do.
 */
public class ArrayHeap<T extends Comparable<T>> implements IPriorityQueue<T> {
    // See spec: you must implement a implement a 4-heap.
    private static final int NUM_CHILDREN = 4;

    // You MUST use this field to store the contents of your heap.
    // You may NOT rename this field: we will be inspecting it within
    // our private tests.
    private T[] heap;

    // Feel free to add more fields and constants.
    
    private int heapSize;
    private static final int DEFAULT_CAPACITY = 10;

    public ArrayHeap() {
        this.heap = makeArrayOfT(DEFAULT_CAPACITY);
        this.heapSize = 0;
    }

    /**
     * This method will return a new, empty array of the given size
     * that can contain elements of type T.
     *
     * Note that each element in the array will initially be null.
     */
    @SuppressWarnings("unchecked")
    private T[] makeArrayOfT(int size) {
        // This helper method is basically the same one we gave you
        // in ArrayDictionary and ChainedHashDictionary.
        //
        // As before, you do not need to understand how this method
        // works, and should not modify it in any way.
        return (T[]) (new Comparable[size]);
    }
    
    /**
     * This method will resize the array used internally to store the heap
     * to a given newSize
     */
    private T[] resizeHeap(int newSize) {
        T[] newHeap = makeArrayOfT(newSize);
        
        // copy elements from old heap to new heap
        for (int i = 0; i < heapSize; i++) {
            newHeap[i] = heap[i];
        }
        
        return newHeap;
    }
    
    /**
     * This method will swap the elements in a given array at the two
     * given integer indices
     */
    private void swapElements(T[] arr, int firstIndex, int secondIndex) {
        T firstElement = arr[firstIndex];
        
        arr[firstIndex] = arr[secondIndex];
        arr[secondIndex] = firstElement;
    }

    @Override
    public T removeMin() {
        if (this.isEmpty()) {
            throw new EmptyContainerException();
        }
        
        // shrink array containing the heap if necessary
        if (heapSize < heap.length / 4) {
            heap = resizeHeap(heap.length / 2);
        }
        
        // move last node to root
        T removedValue = heap[0];
        heap[0] = heap[heapSize - 1];
        heapSize--;
        
        // percolate root node down
        int currIndex = 0;
        int nextIndex = 0;
        do {
            swapElements(heap, currIndex, nextIndex);
            currIndex = nextIndex;
            
            nextIndex = (currIndex * NUM_CHILDREN) + 1;
            for (int i = (currIndex * NUM_CHILDREN) + 1; 
                    i <= Math.min((currIndex * NUM_CHILDREN) + NUM_CHILDREN, heapSize - 1); i++) {
                if (heap[i].compareTo(heap[nextIndex]) < 0) {
                    nextIndex = i;
                }
            }
        } while (currIndex < heapSize && nextIndex < heapSize && heap[nextIndex].compareTo(heap[currIndex]) < 0);
                       
        return removedValue;
    }

    @Override
    public T peekMin() {
        if (this.isEmpty()) {
            throw new EmptyContainerException();
        }
        
        return heap[0];
    }

    @Override
    public void insert(T item) {
        if (item == null) {
            throw new IllegalArgumentException();
        }
        
        // resize array containing the heap if necessary
        if (heapSize >= heap.length) {
            heap = resizeHeap(heap.length * 2);
        }

        // add new element to end of heap array
        heap[heapSize] = item;
        heapSize++;
        
        // percolate new node up
        int currIndex = heapSize-1;
        int nextIndex = heapSize-1;
        do {
            swapElements(heap, currIndex, nextIndex);
            currIndex = nextIndex;
            nextIndex = (currIndex - 1) / NUM_CHILDREN;
        } while (currIndex >= 0 && heap[nextIndex].compareTo(heap[currIndex]) > 0);
        
    }

    @Override
    public int size() {
        return this.heapSize;
    }
}
