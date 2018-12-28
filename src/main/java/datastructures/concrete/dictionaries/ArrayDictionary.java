package datastructures.concrete.dictionaries;

import datastructures.concrete.KVPair;
import datastructures.interfaces.IDictionary;
import misc.exceptions.NoSuchKeyException;

import java.util.Iterator;
import java.util.NoSuchElementException;

/**
 * Replace this class with your ArrayDictionary implementation from project 1.
 */
public class ArrayDictionary<K, V> implements IDictionary<K, V> {
    private Pair<K, V>[] pairs;
    
    private int size = 0;
    
    private static final int DEFAULT_SIZE = 10;

    public ArrayDictionary() {
        this.size = 0;
        this.pairs = makeArrayOfPairs(DEFAULT_SIZE);
    }

    @SuppressWarnings("unchecked")
    private Pair<K, V>[] makeArrayOfPairs(int arraySize) {
        return (Pair<K, V>[]) (new Pair[arraySize]);
    }

    @Override
    public V get(K key) {
        return this.pairs[indexOfKey(key)].value; 
    }

    @Override
    public void put(K key, V value) {
        try {
            int index = indexOfKey(key);
            this.pairs[index].value = value;
        } catch (NoSuchKeyException ex) {
            if (this.size == this.pairs.length) {
                this.pairs = doubleSizeCopyArr(this.pairs);
            }
            this.pairs[this.size] = new Pair<K, V>(key, value);
            this.size++;
        }
    }
    
    @Override
    public V remove(K key) {
        int index = indexOfKey(key);
        V removedValue = this.pairs[index].value;
        this.pairs[index] = this.pairs[this.size - 1];
        this.pairs[this.size - 1] = null;
        this.size--;
        return removedValue;
    }

    @Override
    public boolean containsKey(K key) {
        try {
            indexOfKey(key);
            return true;
        } catch (NoSuchKeyException ex) {
            return false;
        }
    }
    
    @Override
    public int size() {
        return this.size;
    }

    /*
     * Returns a new array with copied contents and double
     * the maximum size
     */
    public Pair<K, V>[] doubleSizeCopyArr(Pair<K, V>[] src) {
        Pair<K, V>[] result = this.makeArrayOfPairs(src.length * 2);
        for (int i = 0; i < src.length; i++) {
            result[i] = src[i];
        }
        return result;
    }
    
    /*
     * Returns index for entry for given key
     * 
     * @throws NoSuchKeyException if the key is not present in the dictionary
     */
    private int indexOfKey(K key) {
        for (int i = 0; i < this.size; i++) {
            if (keysEqual(this.pairs[i].key, key)) {
                return i;
            }
        }
        throw new NoSuchKeyException();
    }
    
    /*
     * Returns true if given keys are equal, false if not
     */
    private boolean keysEqual(K key1, K key2) {
        if (key1 == null || key2 == null) {
            return (key1 == null && key2 == null);
        }
        return key1.equals(key2);
    }

    @Override
    public Iterator<KVPair<K, V>> iterator() {
        return new ArrayDictionaryIterator<>(0, this.size, this.pairs);
    }

    private static class Pair<K, V> {
        public K key;
        public V value;

        // You may add constructors and methods to this class as necessary.
        public Pair(K key, V value) {
            this.key = key;
            this.value = value;
        }

        @Override
        public String toString() {
            return this.key + "=" + this.value;
        }
    }
    
    private static class ArrayDictionaryIterator<K, V> implements Iterator<KVPair<K, V>> {
        private int index;
        private int size;
        private Pair<K, V>[] pairs;
        
        public ArrayDictionaryIterator(int index, int size, Pair<K, V>[] pairs) {
            this.index = index-1;
            this.size = size;
            this.pairs = pairs;
        }

        /**
         * Returns 'true' if the iterator still has elements to look at;
         * returns 'false' otherwise.
         */
        public boolean hasNext() {
            return index < this.size - 1;
        }

        /**
         * Returns the next item in the iteration and internally updates the
         * iterator to advance one element forward.
         *
         * @throws NoSuchElementException if we have reached the end of the iteration and
         *         there are no more elements to look at.
         */
        public KVPair<K, V> next() {
            if (!hasNext()) {
                throw new NoSuchElementException();
            }

            this.index++;
            Pair<K, V> nextPair = this.pairs[this.index];
            return new KVPair<K, V>(nextPair.key, nextPair.value);
        }        
    }
}
