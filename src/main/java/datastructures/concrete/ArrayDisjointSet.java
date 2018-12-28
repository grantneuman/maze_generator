package datastructures.concrete;

import datastructures.concrete.dictionaries.ChainedHashDictionary;
import datastructures.interfaces.IDictionary;
import datastructures.interfaces.IDisjointSet;

/**
 * See IDisjointSet for more details.
 */
public class ArrayDisjointSet<T> implements IDisjointSet<T> {
    // Note: do NOT rename or delete this field. We will be inspecting it
    // directly within our private tests.
    private int[] pointers;

    // However, feel free to add more methods and private helper methods.
    // You will probably need to add one or two more fields in order to
    // successfully implement this class.
    
    private IDictionary<T, Integer> elementIds;
    private IDictionary<Integer, T> elements;
    
    private int forestSize = 0;
    
    private static final int DEFAULT_SIZE = 10;

    public ArrayDisjointSet() {
        pointers = new int[DEFAULT_SIZE];
        elementIds = new ChainedHashDictionary<T, Integer>();
        elements = new ChainedHashDictionary<Integer, T>();
    }
    
    private int[] resizeDisjointSet(int newLen) {
        int[] newArr = new int[newLen];
        
        for (int i = 0; i < Math.min(pointers.length, newLen); i++) {
            newArr[i] = pointers[i];
        }
        
        return newArr;
    }

    @Override
    public void makeSet(T item) {
        if (elementIds.containsKey(item)) {
            throw new IllegalArgumentException();
        }
        
        forestSize++;
        if (forestSize > pointers.length) {
            pointers = resizeDisjointSet(pointers.length * 2);
        }
        
        pointers[forestSize - 1] = -1;
        elements.put(forestSize - 1, item);
        elementIds.put(item, forestSize - 1);
    }

    @Override
    public int findSet(T item) {
        if (!elementIds.containsKey(item)) {
            throw new IllegalArgumentException();
        }
        
        int itemId = elementIds.get(item);
        return findSet(itemId);
    }
    
    private int findSet(int itemIndex) {
        if (pointers[itemIndex] < 0) {
            return itemIndex;
        } else {
            pointers[itemIndex] = findSet(pointers[itemIndex]);
            return pointers[itemIndex];
        }
    }

    @Override
    public void union(T item1, T item2) {        
        int item1Id = findSet(item1);
        int item2Id = findSet(item2);
        
        if (item1Id == item2Id) {
            throw new IllegalArgumentException();
        }
        
        if (pointers[item2Id] < pointers[item1Id]) {
            pointers[item1Id] = item2Id;
        } else if (pointers[item1Id] < pointers[item2Id]) {
            pointers[item2Id] = item1Id;
        } else {
            pointers[item1Id]--;
            pointers[item2Id] = item1Id;
        }
    }
}
