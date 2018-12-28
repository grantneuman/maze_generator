package datastructures.concrete.dictionaries;

import datastructures.concrete.KVPair;
import datastructures.interfaces.IDictionary;

import java.util.Iterator;
import java.util.NoSuchElementException;

/**
 * See IDictionary for details on what each method must do.
 */
public class ChainedHashDictionary<K, V> implements IDictionary<K, V> {
    // You MUST use this field to store the contents of your dictionary.
    // You may NOT rename this field: we will be inspecting it within
    // our private tests.
    //
    // Implementation note: This array should ultimately contain
    // ArrayDictionary instances: each ArrayDictionary should be a
    // "chain" or "bucket" in your hash map.
    private IDictionary<K, V>[] chains;

    // You're encouraged to add extra fields (and helper methods) though!
    
    private int dictionarySize;
    private static final int DEFAULT_CAPACITY = 10;
    private static final double DEFAULT_LOAD_FACTOR = 2;

    public ChainedHashDictionary() {
        this.dictionarySize = 0;
        this.chains = makeArrayOfChains(DEFAULT_CAPACITY);
    }

    /**
     * This method will return a new, empty array of the given size
     * that can contain IDictionary<K, V> objects.
     *
     * Note that each element in the array will initially be null.
     */
    @SuppressWarnings("unchecked")
    private IDictionary<K, V>[] makeArrayOfChains(int size) {
        // Note: You do not need to modify this method.
        // See ArrayDictionary's makeArrayOfPairs(...) method for
        // more background on why we need this method.
        return (IDictionary<K, V>[]) new IDictionary[size];
    }
    
    /**
     * Returns the chain at the index of the hash code of the given key.
     * 
     * Creates a new chain at said index if chain does not exist
     */
    private IDictionary<K, V> getChain(K key) {
        int chainIndex = 0;
        if (key != null) {
            chainIndex = (Math.abs(key.hashCode()) % chains.length);
        }
        
        // add new chain at the position in the dictionary if one doesn't already exist
        if (chains[chainIndex] == null) {
            chains[chainIndex] = new ArrayDictionary<K, V>();
        }
        
        return chains[chainIndex];
    }
    
    /**
     * Returns the given array of dictionary chains resized to the given new length
     */   
    private IDictionary<K, V>[] resize(IDictionary<K, V>[] oldChains, int newLength) {
        IDictionary<K, V>[] newChains = makeArrayOfChains(newLength);
        
        for (KVPair<K, V> pair : this) {
            
            // determines hash code of key with new length
            int newIndex = Math.abs(pair.getKey().hashCode()) % newLength;
            
            // add new chain at the new position if one doesn't already exist
            if (newChains[newIndex] == null) {
                newChains[newIndex] = new ArrayDictionary<K, V>();
            }
            
            // add key-value pair to new array of chains
            newChains[newIndex].put(pair.getKey(), pair.getValue());
        }
        
        return newChains;
    }

    @Override
    public V get(K key) {
        IDictionary<K, V> chain = getChain(key);
        return chain.get(key);
    }
    
    @Override
    public void put(K key, V value) {
        if ((1.0 * dictionarySize / chains.length) >= DEFAULT_LOAD_FACTOR) {
            chains = resize(chains, chains.length * 2);
        }
        
        IDictionary<K, V> chain = getChain(key);
        if (!chain.containsKey(key)) {
            this.dictionarySize++;
        }
        
        chain.put(key, value);
    }

    @Override
    public V remove(K key) {
        int hashIndex = 0;
        if (key != null) {
            hashIndex = Math.abs(key.hashCode()) % chains.length;
        }
        
        IDictionary<K, V> chain = getChain(key);
        V removedValue = chain.remove(key);
        
        if (chain.size() <= 0) {
            chains[hashIndex] = null;
        }
        
        this.dictionarySize--;
        return removedValue;
    }

    @Override
    public boolean containsKey(K key) {
        return getChain(key).containsKey(key);
    }

    @Override
    public int size() {
        return this.dictionarySize;
    }

    @Override
    public Iterator<KVPair<K, V>> iterator() {
        // Note: you do not need to change this method
        return new ChainedIterator<>(this.chains);
    }

    /**
     * Hints:
     *
     * 1. You should add extra fields to keep track of your iteration
     *    state. You can add as many fields as you want. If it helps,
     *    our reference implementation uses three (including the one we
     *    gave you).
     *
     * 2. Before you try and write code, try designing an algorithm
     *    using pencil and paper and run through a few examples by hand.
     *
     *    We STRONGLY recommend you spend some time doing this before
     *    coding. Getting the invariants correct can be tricky, and
     *    running through your proposed algorithm using pencil and
     *    paper is a good way of helping you iron them out.
     *
     * 3. Think about what exactly your *invariants* are. As a
     *    reminder, an *invariant* is something that must *always* be 
     *    true once the constructor is done setting up the class AND 
     *    must *always* be true both before and after you call any 
     *    method in your class.
     *
     *    Once you've decided, write them down in a comment somewhere to
     *    help you remember.
     *
     *    You may also find it useful to write a helper method that checks
     *    your invariants and throws an exception if they're violated.
     *    You can then call this helper method at the start and end of each
     *    method if you're running into issues while debugging.
     *
     *    (Be sure to delete this method once your iterator is fully working.)
     *
     * Implementation restrictions:
     *
     * 1. You **MAY NOT** create any new data structures. Iterators
     *    are meant to be lightweight and so should not be copying
     *    the data contained in your dictionary to some other data
     *    structure.
     *
     * 2. You **MAY** call the `.iterator()` method on each IDictionary
     *    instance inside your 'chains' array, however.
     */
    private static class ChainedIterator<K, V> implements Iterator<KVPair<K, V>> {
        private IDictionary<K, V>[] chains;
        private int index;
        private Iterator<KVPair<K, V>> iterator;
        

        public ChainedIterator(IDictionary<K, V>[] chains) {
            this.chains = chains;
            this.index = nextIndex(0);
            
            if (this.chains[index] == null) {
                this.iterator = null;
            } else {
                this.iterator = this.chains[index].iterator();
            }
        }

        @Override
        public boolean hasNext() {
            int nextIndex = nextIndex(index + 1);
            return iterator != null && 
                    (iterator.hasNext() || (nextIndex <= chains.length-1 && chains[nextIndex] != null));
        }

        @Override
        public KVPair<K, V> next() {
            if (!hasNext()) {
                throw new NoSuchElementException();
            }

            if (!iterator.hasNext()) {
                index = nextIndex(index + 1);
                iterator = chains[index].iterator();
            }
            
            if (!hasNext()) {
                throw new NoSuchElementException();
            }
            
            if (iterator != null && !iterator.hasNext()) {
                throw new NoSuchElementException();
            }
            
            return iterator.next();
        }
        
        /**
         * Returns the next index from the given starting index in the hash table that has a bucket
         * Returns the last index in the hash table if last element does not have a bucket
         */           
        private int nextIndex(int startIndex) {
            while (startIndex + 1 <= chains.length - 1 && chains[startIndex] == null) {
                startIndex++;
            }
            
            return startIndex;
        }
    }
}
